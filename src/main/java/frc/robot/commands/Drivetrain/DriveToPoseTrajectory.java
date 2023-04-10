package frc.robot.commands.Drivetrain;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
  
public class DriveToPoseTrajectory extends CommandBase {
  private final DrivetrainSubsystem drive;
  private final Supplier<Pose2d> poseSupplier;
  private PathPlannerTrajectory trajectory;
  private final Timer timer = new Timer();
  
  private boolean running = false;
  
  PIDController xController = new PIDController(Constants.Drivetrain.kPxAlign, Constants.Drivetrain.kIXController, 0); //FIXME
  PIDController yController = new PIDController(Constants.Drivetrain.kPYAlign, Constants.Drivetrain.kIYController, 0);//FIXME
  PIDController thetaController = new PIDController(
        Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);

  private final PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(xController, yController, thetaController);

  private double driveErrorAbs;
  private double thetaErrorAbs;

  Pose2d currentPose; 
  Pose2d targetPose;

  public DriveToPoseTrajectory(DrivetrainSubsystem drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  public DriveToPoseTrajectory(DrivetrainSubsystem drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI/2, 3* Math.PI/2); // before: (-Math.PI, Math.PI), after: (0, Math,PI)
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(0.2);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    currentPose = drive.getPose();
    targetPose = poseSupplier.get();

    xController.reset();
    yController.reset();
    thetaController.reset();

    // try {
        trajectory = PathPlanner.generatePath(
            new PathConstraints(2.5, 4), 
            new PathPoint(new Translation2d(currentPose.getX(), currentPose.getY()), currentPose.getRotation(), currentPose.getRotation()), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(targetPose.getX(), targetPose.getY()), Rotation2d.fromDegrees(180), targetPose.getRotation()
            )
         );
    // } catch (Exception e) {
    //     System.out.println("Cry");
    // }
    
    timer.restart();
    timer.start();
    
  }

  @Override
  public void execute() {
    // Pose2d currentPose = drive.getPose();
    // Pose2d targetPose = poseSupplier.get();
    SmartDashboard.putNumberArray("Target Coords", new double[]{targetPose.getX(), targetPose.getY()});
    running = true;

    // try{
    PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());

    SmartDashboard.putNumberArray("chassisspeeds", 
      new double[]{
        holonomicDriveController.calculate(drive.getPose(), state).vxMetersPerSecond, 
        holonomicDriveController.calculate(drive.getPose(), state).vyMetersPerSecond
      });

    drive.drive(holonomicDriveController.calculate(drive.getPose(), state));
     
    // }
    // catch(Exception e){
    //     System.out.print("Cry but further in the future");
    // }
    


    // Calculate drive speed
    // double currentDistance =
    //     currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    // driveErrorAbs = currentDistance;
    // double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    // if (driveController.atGoal()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    // double thetaVelocity =
    //     thetaController.calculate(
    //         currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // thetaErrorAbs =
    //     Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.setX();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return atGoal();
    //return /*trajectory == null  || */ timer.hasElapsed(trajectory.getTotalTimeSeconds()) && running/*|| atGoal()*/;
  }
  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && xController.atSetpoint() && thetaController.atSetpoint() && yController.atSetpoint();
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