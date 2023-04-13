// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.swerve.SwerveDriveKinematics2;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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

        public static final double kPXController = 19.2; 

        public static final double kPxAlign = 14;

        public static final double kIXController = 0.000;
        public static final double kPThetaController = 0.075;

        public static final double kPYController = 14.5; // 1
        public static final double kIYController = 0.000;
        public static final double kPYAlign = 11;

        public static final double kPThetaControllerTrajectory = 0.39 * 1.75 * 2 * 1.2; // 0.39 * 2
        public static final double kDThetaControllerTrajectory = 0.004;
        public static final double balanceKP = 0.7;
        public static final double balanceKD = 0.85;
        public static final double balanceGoalDegrees = 0;
        public static final double balanceThresholdDegrees = 1;
        public static final double balanceBackwardsMultiplier = 3.4;
        public static final double balanceDriveTurningDegrees = 2.5;
        public static final double balance = 15;

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
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(95.712));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Front Right Module - Module 1 **/
        public static final class Mod1 {
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 47;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(-82.177));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Back Left Module - Module 2 **/
        public static final class Mod2 {
            public static final int driveMotorID = 32;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(-146.43));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /** Back Right Module - Module 3 **/
        public static final class Mod3 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 41;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.toRadians(322.91 - 180));
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
        public static final double driveKS = (0)/12;
        public static final double driveKV = (0)/12;
        public static final double driveKA = (0)/12;

        public static double balanceKS = 0.5;

        public static double balanceKV = 0.5;

        public static double balanceKA = 0.4;

        public static int allowedMaxAcceleration = 5/2;
        public static double balanceXVelocity = (6 * 0.1524)/1.45; //(6 * 0.1524)/1.45
        public static double balanceTimeout = 1; //1
        // public static class ScoringTag{

        //         private int tagNum;
        //         public ScoringTag(int tagNum){
        //                 if (tagNum < 1 || tagNum > 8) this.tagNum = 0;
        //                 else this.tagNum = tagNum;
        //         }       

        //         public enum ScoringPositions{
        //                 CENTER, RIGHT, LEFT;
        //         }
        // }

        public static enum ScoringTag{
                APRILTAG1, APRILTAG2, APRILTAG3, APRILTAG6, APRILTAG7, APRILTAG8;
        }

        public static final Map<Long, ScoringTag> tagConversion = Map.ofEntries(
                /* i have no clue how to do this w/ deprecating */
                Map.entry(Long.valueOf(1), ScoringTag.APRILTAG1),
                Map.entry(Long.valueOf(2), ScoringTag.APRILTAG2),
                Map.entry(Long.valueOf(3), ScoringTag.APRILTAG3),
                Map.entry(Long.valueOf(6), ScoringTag.APRILTAG6),
                Map.entry(Long.valueOf(7), ScoringTag.APRILTAG7),
                Map.entry(Long.valueOf(8), ScoringTag.APRILTAG8)
        );

        public static final Map<ScoringTag, Pose2d[]> ScoringMap = Map.ofEntries(
                //TODO fill in actual coordinates, degrees, choose how we wanna do the arrays. [left, center, right]?
                /* 8 */
                Map.entry(ScoringTag.APRILTAG8, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        }),

                /* 7 */
                Map.entry(ScoringTag.APRILTAG7, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        }),

                /* 6 */
                Map.entry(ScoringTag.APRILTAG6, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        }),

                /* 3 */
                Map.entry(ScoringTag.APRILTAG3, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        }),

                /* 2 */
                Map.entry(ScoringTag.APRILTAG2, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        }),

                /* 1 */
                Map.entry(ScoringTag.APRILTAG1, 
                        new Pose2d[] {
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
                                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        })
        );
    }

    public static class ClawConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final double TARGET_VELOCITY = 1; //0.875 
        public static final double CUBE_MEDIUM_BOUND = (35); //52 * 0.6 
        public static final double CONE_MEDIUM_BOUND = 60.83; //56.83
}

    public final class GroundIntake{
        public static final int actuator_ID = 57;
        public static final int intake_ID = 22;
        
        public static final int encoder_ID = 40;

        public static final int IR_GATE = 1;
        
        public static final double encoder_offset = 166.03; //223.506

        public static final double x_offset = 0.26;
        public static final double y_offset = 0.4;
        public static final double length = 0.3302;

        public static final double highbound = 5;
        public static final double lowbound = 110;

        public static final double currentLimit = 52; 
        public static final double voltageLimit = 0.2;
        public static final double intakeContinousCurrentLimit = 5;
        public static final double intakePeakCurrentLimit = 10;
    }

    public static Mode getMode() {
        return Mode.REAL;
    }

    public static class VisionConstants {
        /* LL3 Data */
        public static final double H1_LL3 = 0.273; // METERS, 0.3429 on board, 0.489 on floudner.
        public static final double A1_LL3 = 20; // limelight angle!

        public static final double C1_LL3 = 0.323; //vrt / y value check the picture 
        public static final double C2_LL3 = -0.228; // horizontal / x value check the picture :??????
        
        /* LL2 Data */

        public static final double H1_LL2 = 0.279; // METERS, 0.3429 on board, 0.489 on floudner.
        public static final double A1_LL2 = 20; // limelight angle!
 
        public static final double C1_LL2 = 0.314; //vrt / y value check the picture
        public static final double C2_LL2 = 0.244; // horizontal / x value check the picture :??????

        //goals 
        public static final double H2a = 1.1136; //Low goal = 0.628, high goal = 1.17
        public static final double H2b = 0.578; //height of low goal

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

        // TODO replace with actual values
        // Where 0 starts from the corresponding alliance color's right side
        public static final Pose2d INITIAL_TARGET_POSE = new Pose2d(new Translation2d(2.04245, 0.59764), new Rotation2d(-2.96931));
        public static final double Y_OFFSET = 0.5588; // meters
        public static final Pose2d BLUE_INITIAL_TARGET_POSE = new Pose2d(new Translation2d(14.4993, 7.41606), new Rotation2d(-2.96931 + Math.PI / 180));
        public static final double Y_OFFSET_BLUE = -0.5588; // meters
        public static final double ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL = 0.1; // meters
        public static final double ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL = 0.122; // radians
        public static ArrayList<Pose2d> BlueConeScoringPositions = new ArrayList<Pose2d>(
                List.of(
                        new Pose2d(new Translation2d(1.83, 0.502), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 1.6292), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 2.1906), new Rotation2d(0)), 
                        new Pose2d(new Translation2d(1.83, 3.3134), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 3.8748), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 4.9626), new Rotation2d(0))
                )
        );

        public static ArrayList<Pose2d> BlueCubeScoringPositions = new ArrayList<Pose2d>(
                List.of(
                        new Pose2d(new Translation2d(1.83, 1.0678), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 2.752), new Rotation2d(0)),
                        new Pose2d(new Translation2d(1.83, 4.4362), new Rotation2d(0))
                )
        );
        public static ArrayList<Pose2d> RedConeScoringPositions = new ArrayList<Pose2d>(
                List.of(
                        /* just flip x values 16.4846 - 1.83*/ 
                        new Pose2d(new Translation2d(14.6546, 0.502), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 1.6292), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 2.1906), new Rotation2d(Math.PI)), 
                        new Pose2d(new Translation2d(14.6546, 3.3134), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 3.8748), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 4.9626), new Rotation2d(Math.PI))
                )
        );

        public static ArrayList<Pose2d> RedCubeScoringPositions = new ArrayList<Pose2d>(
                List.of(
                        new Pose2d(new Translation2d(14.6546, 1.0678), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 2.752), new Rotation2d(Math.PI)),
                        new Pose2d(new Translation2d(14.6546, 4.4362), new Rotation2d(Math.PI))
                )
        );
    }

    public final class ArmParameters{
        //Positive X Axis defined as the front face of the robot, going from back (battery) to front (roborio)
        //Positive Y axis defined as the axis perpendicular to the ground
        public static final int proximal_left_ID = 23;
        public static final int proximal_right_ID = 52;
        public static final int distal_left_ID = 39;
        public static final int distal_right_ID = 50;
    
        public static final boolean proximal_left_inversion = false;
        public static final boolean proximal_right_inversion = true;
        public static final boolean distal_left_inversion = false;
        public static final boolean distal_right_inversion = true;
    
        public static final int encoder_proximal_left_ID = 12;
        public static final int encoder_proximal_right_ID = 11;
        public static final int encoder_distal_left_ID = 13;
        public static final int encoder_distal_right_ID = 10;
    
        public static final double encoder_proximal_left_offset = 0.67;
        public static final double encoder_proximal_right_offset = 0.227;
        public static final double encoder_distal_left_offset = 0.513;
        public static final double encoder_distal_right_offset = 0.75;
    
        public static final double gear_reduction_proximal = (68.0 / 16) * (68.0 / 8) * (48.0 / 14);
        public static final double gear_reduction_distal = (68.0 / 16) * (68.0 / 8) * (48.0 / 16);
    
        public static final double motor_encoder_ticks_per_revolution = 2048;
    
        public static final double proximal_highbound = 140 * (Math.PI / 180);
        public static final double proximal_lowbound = 40 * (Math.PI / 180);
        public static final double distal_highbound = (20) * (Math.PI / 180);
        public static final double distal_lowbound = (- 180 - 20) * (Math.PI / 180);
    
        public static final double pounds_to_kilograms = 0.453592;
        public static final double inches_to_meters = 0.0254;
        public static final double lbs_sqinches_to_kg_sqmeters = pounds_to_kilograms * inches_to_meters * inches_to_meters;
        
        public static final double proximal_length = 40 * inches_to_meters;
        public static final double proximal_mass = 4*2 *pounds_to_kilograms;
        public static final double proximal_inertia = 2*2961.95 * lbs_sqinches_to_kg_sqmeters;
        public static final double proximal_com = 22.80 * inches_to_meters;
    
        public static final double distal_length = 33 * inches_to_meters;
        public static final double distal_mass = 2.8*2 * pounds_to_kilograms;
        public static final double distal_inertia = 2*866.840 * lbs_sqinches_to_kg_sqmeters;
        public static final double distal_com = 13.56 * inches_to_meters;
      }
      public final class ArmNodeDictionary{
        public double[] ready_highcone_score = new double[]{1.38 + 0.05, 1.18};
        public static final double ready_highcone_score_x = 1.38 + 0.02;
        public static final double ready_highcone_score_y = 1.14; 

        // public static double[] ready_midcone_score = new double[]{0.97, 0.65};
        public static final double ready_midcone_score_x = 0.96 +.1525-0.08 - (3* 0.0254);
        public static final double ready_midcone_score_y = 0.66 +.1525 + (2 * 0.0254);

        public double[] ready_highcube_score = new double[]{1.38, 0.96};
        public static final double ready_highcube_score_x = 1.38;
        public static final double ready_highcube_score_y = 0.96;
        
        public double[] ready_midcube_score = new double[]{0.97, 0.65};
        
        public static final double ready_midcube_score_x = 0.97;
        public static final double ready_midcube_score_y = 0.65;

        public final double[] ready_low_score = new double[]{0.59, 0};
        public static final double ready_low_score_x  = 0.59;
        public static final double ready_low_score_y  = 0;

        public double[] ready_ground_intake = new double[]{0.56, 0.17};
        public static final double ready_ground_intake_x = 0.56;
        public static final double ready_ground_intake_y = 0.17;

        public double[] ground_intake_cube = new double[]{0.60, 0};
        public static final double ground_intake_x = 0.60;
        public static final double ground_intake_y = -0.05;

        public static final double auton_intake_x = 1.45;
        public static final double auton_intake_y = -0.14 - 0.05 - 0.05;

        public double[] ground_intake_cone = new double[]{0.60, 0};
        public static final double ground_intake_cone_x = 0.60;
        public static final double ground_intake_cone_y = -0.15;
    
        public static final double pick_up_ready_position_x = 0.6;
        public static final double pick_up_ready_position_y = 0.3;

        public static final double pick_up_ground_intake_x = 0.55; //0.48
        public static final double pick_up_ground_intake_y = 0.089;

        public double[] ready_arm_score = new double[]{0.688-0.1, 1.05};
        
        public double[] ready_double_substation = new double[]{0.668, 1.08 -0.1524 - 0.0508};
        public static final double ready_double_substation_x = 0.688 -0.1;
        public static final double ready_double_substation_y = 1.08 -0.1524 - 0.0254 -0.0254 - 0.04;
        // public double[] pickup_double_substation = new double[]{0.62, 0.90};
      }

      
}

