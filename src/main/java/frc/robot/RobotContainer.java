// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.SwitchPipeline;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.server.PoseEstimator;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SendableChooser<String> m_autoSwitcher = new SendableChooser<String>();
  public static final String Forward = "Forward";
  public static final String ForwardRotate = "Forward + Rotate";
  public String m_autoSelected;
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();  
  public static final double pi = Math.PI;
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final PoseEstimator localizer = new PoseEstimator(
    m_visionSubsystem, 
    m_drivetrainSubsystem, 
    Constants.Drivetrain.m_kinematics, 
    new Rotation2d(), 
    new Pose2d()
  );
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
  public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_autoSwitcher.addOption(Forward, Forward);
    // m_autoSwitcher.addOption(Rotate, Rotate);
    m_autoSwitcher.addOption(ForwardRotate, ForwardRotate);
    
    // m_field = new Field2d();
    SmartDashboard.putData(m_autoSwitcher);
    
    
    configureButtonBindings();

    configureBindings();
    localizer.start();

    SmartDashboard.putData("Localized robot", field);

    new Thread("Field2D Updater") {
      public void run() {
        while (true) {
          field.setRobotPose(localizer.getPose());
        }
      }
    }.start();
  }

  public void addOne() {
    localizer.addOne();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /** 
   * @param value the joystick input
   * @param exponent the exponent number to use
   * @param activeLinearDeadband when to activate linear scaling instead of exponential scaling
   * **/
  private static double modifyAxis(double value, double exponent, double activeLinearDeadband) {
		// Deadband

		value = MathUtil.applyDeadband(value, 0.05);

    if(Math.abs(value) < activeLinearDeadband)
		  value = Math.copySign(Math.pow(value, exponent), value);
    else if(value > activeLinearDeadband){
      value = Math.copySign(value * value, value);
    }
		return value;
    }

  private void configureButtonBindings() {

    new SequentialCommandGroup( 
        new WaitCommand(1),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(new Pose2d()))
    ).schedule();

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(-m_controller.getLeftX()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_controller.getLeftY()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller.getRightX()/1.5) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            
    ));

    m_LedSubsystem.setDefaultCommand(new DefaultLedCommand(m_LedSubsystem, m_visionSubsystem));

    m_controller.y().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));

    m_controller.leftBumper().whileTrue(
      new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(-m_controller.getLeftX())/3 * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_controller.getLeftY())/3 * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller.getRightX()) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_controller.y().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_controller.x().onTrue(new SwitchPipeline(m_visionSubsystem, 1));
    m_controller.b().onTrue(new SwitchPipeline(m_visionSubsystem, 0));
    m_controller.a().onTrue(new ResetEncoder(m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // String auto = m_autoSwitcher.getSelected();
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", 2, 0.65);
    // switch (auto) {
    //   case Forward:
    //     trajectory = PathPlanner.loadPath("Forward", 0.5, 0.2);
    //   case ForwardRotate:
    //     trajectory = PathPlanner.loadPath("Forward + Rotate", 0.5, 0.2);
    //   // case pathTestBall:
    //     // return getPathTestAuto(trajectory, velocity, acceleration);
    //   default:
    //   trajectory = PathPlanner.loadPath("Forward", 0.5, 0.2);
    // }
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
    
    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();
    
    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
      
    double[] xControllerArray = new double[]{ xController.getP(), xController.getI(), xController.getD()}; 
    double[] yControllerArray = new double[]{ yController.getP(), yController.getI(), yController.getD()}; 
    double[] ThetaControllerArray = new double[]{ thetaController.getP(), thetaController.getI(), thetaController.getD()}; 
    SmartDashboard.putNumberArray("PID X Controller", xControllerArray);
    SmartDashboard.putNumberArray("PID Y Controller", yControllerArray);
    SmartDashboard.putNumberArray("PID Theta Controller",ThetaControllerArray);
    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
        swerveTrajectoryFollower.withTimeout(15).andThen(new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0))))
        );
  }
}
//this is hell
