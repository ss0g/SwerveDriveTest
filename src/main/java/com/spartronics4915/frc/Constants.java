// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc;

import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {
        public static final class FrontLeft {
            public static final int kDriveMotorID = 1;
            public static final int kAngleMotorID = 2;
            public static final int kEncoderID = 0;
            public static final boolean kDriveMotorIsInverted = false;
            public static final boolean kAngleMotorIsInverted = false;
        }

        public static final class FrontRight {
            public static final int kDriveMotorID = 3;
            public static final int kAngleMotorID = 4;
            public static final int kEncoderID = 1;
            public static final boolean kDriveMotorIsInverted = false;
            public static final boolean kAngleMotorIsInverted = false;
        }

        public static final class BackLeft {
            public static final int kDriveMotorID = 5;
            public static final int kAngleMotorID = 6;
            public static final int kEncoderID = 2;
            public static final boolean kDriveMotorIsInverted = false;
            public static final boolean kAngleMotorIsInverted = false;
        }

        public static final class BackRight {
            public static final int kDriveMotorID = 7;
            public static final int kAngleMotorID = 8;
            public static final int kEncoderID = 3;
            public static final boolean kDriveMotorIsInverted = false;
            public static final boolean kAngleMotorIsInverted = false;
        }

        public static final int kPigeonID = 9;

        public static final Function<Integer, CANSparkMax> kMotorConstructor = (Integer ID) -> { return new CANSparkMax(ID, MotorType.kBrushless); };

        public static final double kTrackWidth = 0.75; // meters
        public static final double kWheelBase = 0.75; // meters
        public static final double kChassisRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

        public static final double kDriveMotorGearRatio = 8.33 / 1.0; // driving : driven?? idk
        public static final double kAngleMotorGearRatio = 18.0 / 1.0; // ^

        public static final double kDriveMotorMultiplier = 0.1; // 0.1 for slow testing
        public static final double kAngleMotorMultiplier = 0.1; // 0.1 for slow testing

        public static final double kP = 1;
        public static final double kI = 1;
        public static final double kD = 1;

        private static final double kWheelRadiusInches = 4;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

        private static final double kUnadjustedFreeSpeedFeetPerSecond = 11.9;
        private static final double kUnadjustedFreeSpeedMetersPerSecond = Units.feetToMeters(kUnadjustedFreeSpeedFeetPerSecond);
        private static final double kFreeSpeedAdjustmentMultiplier = 1.0;
        public static final double kFreeSpeedMetersPerSecond = kUnadjustedFreeSpeedMetersPerSecond * kFreeSpeedAdjustmentMultiplier;

        public static final double kMaxOmegaRadiansPerSecond = kFreeSpeedMetersPerSecond / kChassisRadius;
    }

    public static final class OI {
        public static final int kControllerID = 2;

        public static final double kDeadzone = 0.05;
        public static final double kResponseCurveExponent = 5.0 / 3.0;
    }
}
