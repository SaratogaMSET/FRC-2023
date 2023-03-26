
package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.function.Supplier;
import frc.lib.math.GeomUtil;
import frc.robot.Constants;
// import org.littletonrobotics.frc2023.util.LoggedTunableNumber;

public class DriveToPose extends CommandBase {
  private final DrivetrainSubsystem drive;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private double driveErrorAbs;
  private double thetaErrorAbs;

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

  public DriveToPose(DrivetrainSubsystem drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  public DriveToPose(DrivetrainSubsystem drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(0, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(1.3, 0.7));
      driveController.setTolerance(0.1);
      thetaController.setP(4);
      thetaController.setD(0.0);
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(4, 1));
      thetaController.setTolerance(0.1);
    

    // Get current and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
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

    drive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.setX();
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

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}