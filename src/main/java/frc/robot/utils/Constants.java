package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class WristConstants{

        // TODO: ADJUST CONSTANTS
        public static final int wristMotorId = 25;
        public static final int wristEncoderId = 24;
        public static final int wristEncoderOffset = 0;

    }

    public static final class IntakeConstants{

        // TODO: ADJUST CONSTANTS
        public static final int intakeMotorId = 30;
        public static final int intakeMotorStallCurrentLimit = 40;
        public static final int intakeMotorFreeSpinCurrentLimit = 40;
        public static final boolean intakeMotorInverted = false;
        public static final double openLoopRampRate = 1.0;
    }

    public static final class ArmConstants{

        // TODO: ADJUST CONSTANTS
        public static final int rotateMotorId = 20;
        public static final int extensionMotorId = 22;
        public static final int rotateCanCoderId = 21;

        public static final boolean rotateMotorInverted = false;
        public static final boolean extensionMotorInverted = false;
        public static final boolean rotateCanCoderReversed = false;

        public static final int rotateCanCoderOffset = 0;
        
        public static final int rotateMotorCurrentLimit = 50;
        public static final int extensionMotorCurrentLimit = 50;

        public static final double rotateMotorOpenLoopRampRate = 1.0;
        public static final double extensionMotorOpenLoopRampRate = 1.0;

        public static final double rotatekP = 0.5;
        public static final double rotatekI = 0;
        public static final double rotatekD = 0;
        public static final double rotatekIz = 0;
        public static final double rotatekFF = 0;
        public static final double rotateMaxOutput = 1;
        public static final double rotateMinOutput = -1;
        public static final double allowedErr = 0.002; // Rotations, not radians
        // Smart motion constants
        public static final double rotateMaxVel = 1500; // RPM
        public static final double rotateMinVel = 0;
        public static final double rotateMaxAcc = 1000;

        public static final double gravityFF = 0.1;

        public static final double extensionkP = 0.5;
        public static final double extensionkI = 0;
        public static final double extensionkD = 0;
        public static final double extensionkIz = 0;
        public static final double extensionkFF = 0;
        public static final double extensionMaxOutput = 1;
        public static final double extensionMinOutput = -1;
        public static final double extensionAllowedErr = 0.002;
        public static final double extensionEncoderConversionFactor = 0.65625; // 0.65625 rotations per inch

        public static final double extensionMaxVel = 1500;
        public static final double extensionMinVel = 0;
        public static final double extensionMaxAcc = 1000;
    }

    public static final class DriveConstants{

        public static final double kWheelBase = 23.5;
        public static final double kTrackWidth = 16;

        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kMaxAccelerationRateUnitsPerSecond = 5.0;
        public static final double kMaxTurningRateUnitsPerSecond = 5.0;

        public static final double kDriveMaxSpeedMetersPerSecond = 1;
        public static final double kDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    }

    public static final class ModuleConstants{

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.16;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

        public static final double kModuleP = 0.5;
        public static final double kModuleI = 0;
        public static final double kModuleD = 0;

        public static final class FrontLeftModule{
            public static final int driveMotorId = 04;
            public static final int turnMotorId = 05;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 12;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final boolean absoluteEncoderReversed = false;
            public static final String name = "Front Left";
        }

        public static final class FrontRightModule{
            public static final int driveMotorId = 00;
            public static final int turnMotorId = 01;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 10;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final boolean absoluteEncoderReversed = false;
            public static final String name = "Front Right";
        }

        public static final class BackLeftModule{
            public static final int driveMotorId = 06;
            public static final int turnMotorId = 07;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 13;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final boolean absoluteEncoderReversed = false;
            public static final String name = "Back Left";
        }

        public static final class BackRightModule{
            public static final int driveMotorId = 02;
            public static final int turnMotorId = 03;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 11;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final boolean absoluteEncoderReversed = false;
            public static final String name = "Back Right";
        }

    }

    public static final class IOConstants{

        public static final double kDeadband = 0.05;

        public static final class ButtonBoxButtons{
            public static final int straightUpButton = 1;
            public static final int armRotateButton = 2;
            public static final int frontSwitch = 3;
            public static final int floorSwitch = 4;
            public static final int highSwitch = 5;
            public static final int tippedSwitch = 6;
            public static final int cubeSwitch = 7;
            public static final int station2Button = 8;
            public static final int parallelButton = 9;
            public static final int station1Button = 10;
            public static final int jogDownSwitch = 11;
            public static final int jogUpSwitch = 12;
            public static final int knobAxis = 0;
        }

    }
}