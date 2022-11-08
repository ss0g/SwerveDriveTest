package com.spartronics4915.frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.spartronics4915.frc.Constants.Drive.FrontLeft;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.PigeonIMU;

import static com.spartronics4915.frc.Constants.Drive.*;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static enum ModuleLocation {
        FL(kWheelBase / 2, kTrackWidth / 2),
        FR(kWheelBase / 2, -kTrackWidth / 2),
        BL(-kWheelBase / 2, kTrackWidth / 2),
        BR(-kWheelBase / 2, -kTrackWidth / 2);

        private final Translation2d mLocation;
        
        /** x = forwards, y = left */
        private ModuleLocation(double x, double y) {
            mLocation = new Translation2d(x, y);
        }

        public Translation2d getLocation() {
            return mLocation;
        }

        @Override
        public String toString() {
            switch(this) {
                case FL: return "Front Left";
                case FR: return "Front Right";
                case BL: return "Back Left";
                case BR: return "Back Right";
                default: return "";
            }
        }
    }
    
    private class SwerveModule {
        private final CANSparkMax mDriveMotor;
        private final CANSparkMax mAngleMotor;
        private final AnalogEncoder mEncoder;

        private final ModuleLocation mModuleLocation;
        
        private SwerveModuleState mState;
        private Rotation2d mCurrentRotation;

        private final PIDController mPIDController;

        public SwerveModule(ModuleLocation moduleLocation) {
            int driveMotorID;
            int angleMotorID;
            int encoderID;

            boolean driveInverted;
            boolean angleInverted;

            switch(moduleLocation) {
                case FL: {
                    driveMotorID = FrontLeft.kDriveMotorID;
                    angleMotorID = FrontLeft.kAngleMotorID;
                    encoderID = FrontLeft.kEncoderID;
                    driveInverted = FrontLeft.kDriveMotorIsInverted;
                    angleInverted = FrontLeft.kAngleMotorIsInverted;
                    break;
                }
                case FR: {
                    driveMotorID = FrontRight.kDriveMotorID;
                    angleMotorID = FrontRight.kAngleMotorID;
                    encoderID = FrontRight.kEncoderID;
                    driveInverted = FrontRight.kDriveMotorIsInverted;
                    angleInverted = FrontRight.kAngleMotorIsInverted;
                    break;
                }
                case BL: {
                    driveMotorID = BackLeft.kDriveMotorID;
                    angleMotorID = BackLeft.kAngleMotorID;
                    encoderID = BackLeft.kEncoderID;
                    driveInverted = BackLeft.kDriveMotorIsInverted;
                    angleInverted = BackLeft.kAngleMotorIsInverted;
                    break;
                }
                case BR: {
                    driveMotorID = BackRight.kDriveMotorID;
                    angleMotorID = BackRight.kAngleMotorID;
                    encoderID = BackRight.kEncoderID;
                    driveInverted = BackRight.kDriveMotorIsInverted;
                    angleInverted = BackRight.kAngleMotorIsInverted;
                    break;
                }
                default: {
                    driveMotorID = -1;
                    angleMotorID = -1;
                    encoderID = -1;
                    driveInverted = false;
                    angleInverted = false;
                }
            }

            mDriveMotor = kMotorConstructor.apply(driveMotorID);
            mAngleMotor = kMotorConstructor.apply(angleMotorID);
            mEncoder = new AnalogEncoder(encoderID);

            mDriveMotor.setInverted(driveInverted);
            mAngleMotor.setInverted(angleInverted);
            mDriveMotor.setIdleMode(IdleMode.kBrake);
            mAngleMotor.setIdleMode(IdleMode.kBrake);
            mEncoder.setDistancePerRotation(2 * Math.PI); // using radians as units
            mEncoder.reset();

            mModuleLocation = moduleLocation;
            mState = new SwerveModuleState();

            mPIDController = new PIDController(kP, kI, kD);
        }

        // TODO: fix calculations for angle motor output
        public void drive() {
            mCurrentRotation = new Rotation2d(mEncoder.getDistance());
            SwerveModuleState state = SwerveModuleState.optimize(mState, mCurrentRotation);
            mPIDController.setSetpoint(state.angle.getRadians());
            double angleMotorOutput = MathUtil.clamp(mPIDController.calculate(mCurrentRotation.getRadians()), -1, 1);

            SmartDashboard.putNumber(mModuleLocation.toString() + "yaw (degrees)", mCurrentRotation.getDegrees());
            SmartDashboard.putNumber(mModuleLocation.toString() + "module speed output", state.speedMetersPerSecond / kFreeSpeedMetersPerSecond);
            SmartDashboard.putNumber(mModuleLocation.toString() + "module angle output", angleMotorOutput);

            mDriveMotor.set(state.speedMetersPerSecond / kFreeSpeedMetersPerSecond);
            mAngleMotor.set(0); // set to 0 because output calculations are wrong
        }

        public void setState(SwerveModuleState state) {
            mState = state;
        }
        
        public Translation2d getLocation() {
            return mModuleLocation.getLocation();
        }

        public void setDriveMotor(double x) {
            mDriveMotor.set(x);
        }

        public void setAngleMotor(double x) {
            mAngleMotor.set(x);
        }
    }
    
    private final SwerveDriveKinematics mKinematics;

    private final SwerveModule mFrontLeftModule;
    private final SwerveModule mFrontRightModule;
    private final SwerveModule mBackLeftModule;
    private final SwerveModule mBackRightModule;

    private final PigeonIMU mIMU;

    public SwerveDriveSubsystem() {
        mFrontLeftModule = new SwerveModule(ModuleLocation.FL);
        mFrontRightModule = new SwerveModule(ModuleLocation.FR);
        mBackLeftModule = new SwerveModule(ModuleLocation.BL);
        mBackRightModule = new SwerveModule(ModuleLocation.BR);

        mIMU = new PigeonIMU(kPigeonID);

        mKinematics = new SwerveDriveKinematics(
            mFrontLeftModule.getLocation(),
            mFrontRightModule.getLocation(),
            mBackLeftModule.getLocation(),
            mBackRightModule.getLocation()
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                new Rotation2d(Units.degreesToRadians(-mIMU.getYaw())));
        } else {
            chassisSpeeds = new ChassisSpeeds(
                translation.getX() * kFreeSpeedMetersPerSecond,
                translation.getY() * kFreeSpeedMetersPerSecond,
                -rotation * kMaxOmegaRadiansPerSecond // negative if ccw is positive
            );
        }

        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);

        mFrontLeftModule.setState(moduleStates[0]);
        mFrontRightModule.setState(moduleStates[1]);
        mBackLeftModule.setState(moduleStates[2]);
        mBackRightModule.setState(moduleStates[3]);

        mFrontLeftModule.drive();
        mFrontRightModule.drive();
        mBackLeftModule.drive();
        mBackRightModule.drive();
    }

    /** temporary */
    public void testDrive() {
        mFrontLeftModule.setDriveMotor(0.5);
        mFrontLeftModule.setAngleMotor(0.5);
    }
}
