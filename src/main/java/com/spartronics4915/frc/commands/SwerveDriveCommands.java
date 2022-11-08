package com.spartronics4915.frc.commands;

import com.spartronics4915.frc.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.spartronics4915.frc.Constants.Drive.*;
import static com.spartronics4915.frc.Constants.OI.*;

public class SwerveDriveCommands {
    private final SwerveDriveSubsystem mSwerve;

    private final XboxController mController;

    public SwerveDriveCommands(SwerveDriveSubsystem swerve, XboxController controller) {
        mSwerve = swerve;

        mController = controller;
    }

    // temporary
    public Command getCommand() {
        return new InstantCommand(mSwerve::testDrive, mSwerve);
    }

    public class TeleopCommand extends CommandBase {
        public TeleopCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        // TODO: add a button to toggle field-oriented driving mode
        @Override
        public void execute() {
            double x1 = mController.getLeftX();
            double y1 = -mController.getLeftY(); // convention is to invert y
            double x2 = mController.getRightX();

            x1 = applyTransformations(x1);
            y1 = applyTransformations(y1);
            x2 = applyTransformations(x2);

            Translation2d translation = new Translation2d(y1, -x1);
            double rotation = -x2;

            mSwerve.drive(translation, rotation, false);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    private double applyTransformations(double x) {
        return applyResponseCurve(applyDeadzone(x));
    }

    private double applyDeadzone(double x) {
        return Math.abs(x) > kDeadzone ? x : 0;
    }

    private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(Math.abs(x), kResponseCurveExponent);
    }
}
