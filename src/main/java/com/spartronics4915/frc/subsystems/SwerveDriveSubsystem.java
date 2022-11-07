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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import static com.spartronics4915.frc.Constants.Drive.*;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static enum ModuleLocation {
        FL,
        FR,
        BL,
        BR;
        
        private ModuleLocation() {}
    }
    
    private class SwerveModule {
        private final CANSparkMax mDriveMotor;
        private final CANSparkMax mAngleMotor;
        private final AnalogEncoder mEncoder;

        private final Translation2d mLocation;
        
        private SwerveModuleState mState;
        private Rotation2d mCurrentRotation;

        private final PIDController mPIDController;

        public SwerveModule(ModuleLocation loc) {
            int driveMotorID;
            int angleMotorID;
            int encoderID;

            Translation2d location;
            SwerveModuleState state;

            switch(loc) {
                case FL: {
                    driveMotorID = FrontLeft.kDriveMotorID;
                    angleMotorID = FrontLeft.kAngleMotorID;
                    encoderID = FrontLeft.kEncoderID;
                    location = new Translation2d(-kTrackWidth, kWheelBase);
                    break;
                }
                case FR: {
                    driveMotorID = FrontRight.kDriveMotorID;
                    angleMotorID = FrontRight.kAngleMotorID;
                    encoderID = FrontRight.kEncoderID;
                    location = new Translation2d(kTrackWidth, kWheelBase);
                    break;
                }
                case BL: {
                    driveMotorID = BackLeft.kDriveMotorID;
                    angleMotorID = BackLeft.kAngleMotorID;
                    encoderID = BackLeft.kEncoderID;
                    location = new Translation2d(-kTrackWidth, -kWheelBase);
                    break;
                }
                case BR: {
                    driveMotorID = BackRight.kDriveMotorID;
                    angleMotorID = BackRight.kAngleMotorID;
                    encoderID = BackRight.kEncoderID;
                    location = new Translation2d(kTrackWidth, -kWheelBase);
                    break;
                }
                default: {
                    driveMotorID = -1;
                    angleMotorID = -1;
                    encoderID = -1;
                    location = new Translation2d();
                }
            }

            mDriveMotor = kMotorConstructor.apply(driveMotorID);
            mAngleMotor = kMotorConstructor.apply(angleMotorID);
            mEncoder = new AnalogEncoder(encoderID);

            mDriveMotor.setIdleMode(IdleMode.kBrake);
            mAngleMotor.setIdleMode(IdleMode.kBrake);
            mEncoder.setDistancePerRotation(2 * Math.PI); // using radians as units

            mLocation = location;
            mState = new SwerveModuleState();

            mPIDController = new PIDController(kP, kI, kD);
        }

        // TODO: fix calculations for angle motor output
        public void drive() {
            mCurrentRotation = new Rotation2d(mEncoder.getDistance());
            SwerveModuleState state = SwerveModuleState.optimize(mState, mCurrentRotation);
            mPIDController.setSetpoint(state.angle.getRadians());
            double angleMotorOutput = MathUtil.clamp(mPIDController.calculate(mCurrentRotation.getRadians()), -1, 1);

            mDriveMotor.set(state.speedMetersPerSecond / kFreeSpeedMetersPerSecond);
            mAngleMotor.set(0); // set to 0 because output calculations are wrong
        }

        public Translation2d getLocation() {
            return mLocation;
        }

        public void setState(SwerveModuleState state) {
            mState = state;
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

    public SwerveDriveSubsystem() {
        mFrontLeftModule = new SwerveModule(ModuleLocation.FL);
        mFrontRightModule = new SwerveModule(ModuleLocation.FR);
        mBackLeftModule = new SwerveModule(ModuleLocation.BL);
        mBackRightModule = new SwerveModule(ModuleLocation.BR);

        mKinematics = new SwerveDriveKinematics(
            mFrontLeftModule.getLocation(),
            mFrontRightModule.getLocation(),
            mBackLeftModule.getLocation(),
            mBackRightModule.getLocation()
        );
    }

    public void drive(double x1, double y1, double x2) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            y1 * kFreeSpeedMetersPerSecond,
            -x1 * kFreeSpeedMetersPerSecond,
            -x2 * kMaxOmegaRadiansPerSecond // i think this should be negative?? is chassisspeeds omega cw or ccw positive
        );

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
