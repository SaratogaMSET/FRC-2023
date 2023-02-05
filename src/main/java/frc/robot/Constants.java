// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean competitionMode = false;
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
    }

    public static class Drivetrain {
        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);
        
        public static final boolean invertGyro = true; //Nav_x vs. Pigeon
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
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.7435; 
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4381;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 32;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 33;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 43;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(25.0);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 34;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 35;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 45;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-20.0);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 41;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(40.0);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 36;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 37;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 47;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(170.0);

        public static final double kPXController = 0.0; //0.6
        public static final double kIXController = 0.000;
        public static final double kPThetaController = 0.075;

        public static final double kPYController = 0.0; //1
        public static final double kIYController = 0.000;
        public static final double kPThetaControllerTrajectory = 0.0; //0.39 * 2
        public static final double balanceKP = 0.015; // P (Proportional) constant of a PID loop
        public static final double balanceGoalDegrees = 0;
        public static final double balanceThresholdDegrees = 1;
        public static final double balanceBackwardsMultiplier = 1.35;
        public static final double balanceDriveTurningDegrees = 3;
        
        public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 1.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 1.0),
          // Front right
          new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 1.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 1.0),
          // Back left
          new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 1.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 1.0),
          // Back right
          new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 1.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 1.0)
  );
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Coast;
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 32;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(25.0+44.7-90));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 34;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 45;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(-20.0 -322.01));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 41;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(40.0-80.65));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 47;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(170.0-23.76-46.14+90));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);
    }
        
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class VisionConstants {
        public static final double H1 = 0.3429; // METERS, 0.3429 on board, 0.489 on floudner.
        public static final double H2 = 1.1736; //Low goal = 0.628, high goal = 1.17
        public static final double A1 = 0;
        
        public static final double C1 = 0; //vrt / y value check the picture Idk why govind called it that
        public static final double C2 = 0; // horizontal / x value check the picture :??????

        /* public static class Field {
        public static final float FIELD_WIDTH = 16.54175f;
        public static final float FIELD_HEIGHT = 8.0137f;
        public static final int NUM_TAGS = 8;
        public static final Point[] TAGS = {
            new Point(Tags.ID_1.x, Tags.ID_1.y),
            new Point(Tags.ID_2.x, Tags.ID_2.y),
            new Point(Tags.ID_3.x, Tags.ID_3.y),
            new Point(Tags.ID_4.x, Tags.ID_4.y),
            new Point(Tags.ID_5.x, Tags.ID_5.y),
            new Point(Tags.ID_6.x, Tags.ID_6.y),
            new Point(Tags.ID_7.x, Tags.ID_7.y),
            new Point(Tags.ID_8.x, Tags.ID_8.y)
        };
        } */

        public static enum Tags {
            ID_1(15.513558f, 1.071626f, 0.462788f),
            ID_2(15.513558f, 2.748026f, 0.462788f),
            ID_3(15.513558f, 4.424426f, 0.462788f),
            ID_4(16.178784f, 6.749796f, 0.695452f),
            ID_5(0.36195f, 6.749796f, 0.695452f),
            ID_6(1.02743f, 4.424426f, 0.462788f),
            ID_7(1.02743f, 2.748026f, 0.462788f),
            ID_8(1.02743f, 1.071626f, 0.462788f);

            public final float x;
            public final float y;
            public final float z;

            Tags(float x, float y, float z) {
                this.x = x;
                this.y = y;
                this.z = z;
            }
        }

        public static enum FieldZones {
            ZONE_1(0, 0, 0, 0);

            private final double minX;
            private final double minY;
            private final double maxX;
            private final double maxY;

            FieldZones(double minX, double minY, double maxX, double maxY) {
                this.minX = minX;
                this.minY = minY;
                this.maxX = maxX;
                this.maxY = maxY;
            }

            public double getMinX() {
                return minX;
            }
            
            public double getMinY() {
                return minY;
            }
            
            public double getMaxX() {
                return maxX;
            }
            
            public double getMaxY() {
                return maxY;
            }
        }
    }

    public static class FilterConstants {
        public static final int NUM_PARTICLES = 1;
    }
}
