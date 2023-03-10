// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.swerve.SwerveDriveKinematics2;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean competitionMode = false;
    public static final Mode currentMode = Mode.REAL;
    public static final boolean tuningMode = true;
    public static final double loopPeriodSecs = 0.02;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class Drivetrain {
        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        public static final boolean invertGyro = true; // Nav_x vs. Pigeon
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4953;

        public static final double kPXController = 19.2; // 0.6
        public static final double kIXController = 0.000;
        public static final double kPThetaController = 0.075;

        public static final double kPYController = 14.5; // 1
        public static final double kIYController = 0.000;
        public static final double kPThetaControllerTrajectory = 0.39 * 1.75 * 2 * 1.2; // 0.39 * 2
        public static final double kDThetaControllerTrajectory = 0.004;
        public static final double balanceKP = 3.5;
        public static final double balanceGoalDegrees = 0;
        public static final double balanceThresholdDegrees = 1;
        public static final double balanceBackwardsMultiplier = 1.8;
        public static final double balanceDriveTurningDegrees = 2.5;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public final static SwerveDriveKinematics2 m_kinematics2 = new SwerveDriveKinematics2(
                // Front left
                new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0));
        public final static SwerveDriveKinematics m_kinematics1 = new SwerveDriveKinematics(
                // Front left
                new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0));
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /** Front Left Module - Module 0 **/
        public static final class Mod0 {
            public static final int driveMotorID = 34;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 45;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(34.45 + 180));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Front Right Module - Module 1 **/
        public static final class Mod1 {
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 47;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(357.19 + 180));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Back Left Module - Module 2 **/
        public static final class Mod2 {
            public static final int driveMotorID = 32;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(312.45));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Back Right Module - Module 3 **/
        public static final class Mod3 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 41;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(349.54));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 25;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 30;
        public static final int drivePeakCurrentLimit = 45;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0);
        public static final double driveKV = (0);
        public static final double driveKA = (0);
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final double GEAR_RATIO = 1 / 20.0;
        public static final int HALL_EFFECT = 0;
        public static final double TARGET_VELOCITY = 0.2;
        public static final int INTAKE_DISTANCE_THRESHOLD = 50;
        public static final double TORQUE_CONSTANT = 0.01042;
        public static final double RESISTANCE = 12 / (11000 * 2 * 3.14159265 / 60);
        public static final double TARGET_VOLTAGE = 5.0;
        public static final double TORQUE_THRESHOLD = 125;
        public static final double CLOSING_TORQUE_THRESHOLD = 75;
        public static final double LOWER_BOUND = -2.5;
        public static final double CUBE_MEDIUM_BOUND = 2.5;
        public static final double CONE_MEDIUM_BOUND = 4.6;
        public static final double UPPER_BOUND = 22;
        public static final double HOLD_VOLTAGE = TARGET_VOLTAGE / 10;
        public static final double AUTO_CONE_DIAMETER = 0.0;
        public static final double AUTO_CUBE_DIAMETER = 0.0;
        public static final double AUTO_KP = 0.0;
        public static final double AUTO_KI = 0.0;
        public static final double AUTO_KD = 0.0;
    }

    public static Mode getMode() {
        return Mode.REAL;
    }

    public static class VisionConstants {
        /* LL3 Data */
        public static final double H1_LL3 = 0.273; // METERS, 0.3429 on board, 0.489 on floudner.
        public static final double A1_LL3 = 20; // limelight angle!

        public static final double C1_LL3 = 0.323; //vrt / y value check the picture Idk why govind called it that
        public static final double C2_LL3 = 0.228; // horizontal / x value check the picture :??????
        
        /* LL2 Data */

        public static final double H1_LL2 = 0.279; // METERS, 0.3429 on board, 0.489 on floudner.
        public static final double A1_LL2 = 20; // limelight angle!

        public static final double C1_LL2 = 0.314; //vrt / y value check the picture Idk why govind called it that
        public static final double C2_LL2 = 0.244; // horizontal / x value check the picture :??????

        //goals 
        public static final double H2a = 1.1736; //Low goal = 0.628, high goal = 1.17
        public static final double H2b = 0.638; //height of low goal

    }

    public static class Vision {
        public static final int LED = 3;
        public static final double H1 = 5; // distance between limelight and ground (height of limelight mount)
        public static final double H2 = 107; // height of target
        public static final double A1 = 35; // angle from horizontal axis
        public static final double AREA_VISIBLE = 1; //if area is large enough to be visible (ta)
        
        public static class Distance {
            public static final int STATE1 = 110; 
            public static final int STATE2 = 150;
            public static final int STATE3 = 190;
            public static final int STATE4 = 270;
        }

        public static final double apriltagToMidHorizontal = 0.219837;
        public static final double apriltagToHighHorizontal = apriltagToMidHorizontal*3;
        public static final double apriltagtoMidVertical = 0.15875;
        public static final double apriltagtoHighVertical = 0.73025;

    }
    
    }

