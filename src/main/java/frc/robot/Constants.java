// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.swerve.SwerveDriveKinematics2;
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
    public static final Mode currentMode = Mode.REAL;
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
      public static final COTSFalconSwerveConstants chosenModule = 
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
      
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4381; 
      /**
       * The front-to-back distance between the drivetrain wheels.
       *
       * Should be measured from center to center.
       */
      public static final double DRIVETRAIN_WHEELBASE_METERS =  0.7435;

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

      public static final double kPXController = 0.6; //0.6
      public static final double kIXController = 0.000;
      public static final double kPThetaController = 0.075;

      public static final double kPYController = 1; //1
      public static final double kIYController = 0.000;
      public static final double kPThetaControllerTrajectory = 0.39*1.5; //0.39 * 2
      public static final double kDThetaControllerTrajectory = 0.004;
      public static final double balanceKP = 0.015; 
      public static final double balanceGoalDegrees = 0;
      public static final double balanceThresholdDegrees = 1;
      public static final double balanceBackwardsMultiplier = 1.35;
      public static final double balanceDriveTurningDegrees = 3;


      public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

      public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

      public final static SwerveDriveKinematics2 m_kinematics = new SwerveDriveKinematics2(
        // Front left
        new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
);
      public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
      public static final NeutralMode driveNeutralMode = NeutralMode.Coast;
      public static final boolean canCoderInvert = chosenModule.canCoderInvert;

      /** Front Left Module - Module 0**/
      public static final class Mod0 { 
          public static final int driveMotorID = 32;
          public static final int angleMotorID = 33;
          public static final int canCoderID = 43;
          public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(25.0+44.7-90));
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      /** Front Right Module - Module 1**/
      public static final class Mod1 { 
          public static final int driveMotorID = 34;
          public static final int angleMotorID = 35;
          public static final int canCoderID = 45;
          public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(-20.0 -322.01));
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /** Back Left Module - Module 2 **/
      public static final class Mod2 { 
          public static final int driveMotorID = 30;
          public static final int angleMotorID = 31;
          public static final int canCoderID = 41;
          public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.toRadians(40.0-80.65 - 62.4+61.62));
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /** Back Right Module - Module 3 **/
      public static final class Mod3 { 
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
      public static final double driveKP = 0.05; 
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values 
       * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      public static final double driveKS = (0.32 / 12); 
      public static final double driveKV = (1.51 / 12); 
      public static final double driveKA = (0.27 / 12);
  }
}
