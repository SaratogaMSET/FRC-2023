// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.GeomUtil;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class DriveToPose extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          3, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 2.0), Constants.loopPeriodSecs);
    // theta means angle btw
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 1.75), Constants.loopPeriodSecs);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  // private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/DriveKp");
  // private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/DriveKd");
  // private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/ThetaKp");
  // private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/ThetaKd");
  // private static final LoggedTunableNumber driveMaxVelocity =
  //     new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  // private static final LoggedTunableNumber driveMaxAcceleration =
  //     new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  // private static final LoggedTunableNumber thetaMaxVelocity =
  //     new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  // private static final LoggedTunableNumber thetaMaxAcceleration =
  //     new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  // private static final LoggedTunableNumber driveTolerance =
  //     new LoggedTunableNumber("DriveToPose/DriveTolerance");
  // private static final LoggedTunableNumber thetaTolerance =
  //     new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  // private static final LoggedTunableNumber ffMinRadius =
  //     new LoggedTunableNumber("DriveToPose/FFMinRadius");
  // // private static final LoggedTunableNumber ffMaxRadius =
  // //     new LoggedTunableNumber("DriveToPose/FFMinRadius");

  private static final double ffMinRadius = 0.2;
  private static final double ffMaxRadius = 0.6;

  // static {
  //   switch (Constants.getRobot()) {
  //     case ROBOT_2023:
  //     case ROBOT_SIMBOT:
  //       driveKp.initDefault(2.0);
  //       driveKd.initDefault(0.0);
  //       thetaKp.initDefault(5.0);
  //       thetaKd.initDefault(0.0);
  //       driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
  //       driveMaxAcceleration.initDefault(Units.inchesToMeters(120.0));
  //       thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
  //       thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
  //       driveTolerance.initDefault(0.01);
  //       thetaTolerance.initDefault(Units.degreesToRadians(1.0));
  //       ffMinRadius.initDefault(0.2);
  //       ffMaxRadius.initDefault(0.8);
  //     default:
  //       break;
  //   }
  // }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainSubsystem drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainSubsystem drive, Supplier<Pose2d> poseSupplier) {
    this.drivetrainSubsystem = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    // basically finds the allows the pid to not spin like 350 degrees to get to a point when it could just spin 10. 
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drivetrainSubsystem.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drivetrainSubsystem.getFieldVelocity().dx, drivetrainSubsystem.getFieldVelocity().dy)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drivetrainSubsystem.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), drivetrainSubsystem.getFieldVelocity().dtheta);
    lastSetpointTranslation = drivetrainSubsystem.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers. Basically u can change pid values from the dashboard! A thing for convenience. 
    // if (driveKp.hasChanged(hashCode())
    //     || driveKd.hasChanged(hashCode())
    //     || thetaKp.hasChanged(hashCode())
    //     || thetaKd.hasChanged(hashCode())) {
    //   driveController.setP(driveKp.get());
    //   driveController.setD(driveKd.get());
    //   driveController.setConstraints(
    //       new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    //   driveController.setTolerance(driveTolerance.get());
    //   thetaController.setP(thetaKp.get());
    //   thetaController.setD(thetaKd.get());
    //   thetaController.setConstraints(
    //       new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    //   thetaController.setTolerance(thetaTolerance.get());
    // }

    // Get current and target pose
    var currentPose = drivetrainSubsystem.getPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    // Magnitude of distance between pose and target(hypotenuse)
    double currentDistance =
        currentPose.getTranslation().getDistance(targetPose.getTranslation()); 

    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
            0.0,
            1.0);

    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0); // P control "drives error to zero"
    if (driveController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
            drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.getInstance().recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.getInstance()
        .recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.getInstance()
        .recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.getInstance()
        .recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.getInstance()
        .recordOutput(
            "Odometry/DriveToPoseSetpoint",
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.getInstance().recordOutput("Odometry/DriveToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drivetrainSubsystem.setX();
    Logger.getInstance().recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
    Logger.getInstance().recordOutput("Odometry/DriveToPoseGoal", new double[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  @Override
  public boolean isFinished(){
    return atGoal();
  }
  
  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}