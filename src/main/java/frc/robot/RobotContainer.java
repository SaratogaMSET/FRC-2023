// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.CANdle.ToggleLEDCommand;
import frc.robot.commands.Drivetrain.AlignToCone;
import frc.robot.commands.Drivetrain.BalanceCommand;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.MoveWithClosest90;
import frc.robot.commands.Drivetrain.TurnTo90;
import frc.robot.commands.Drivetrain.AlignToCone;
import frc.robot.commands.Drivetrain.ZeroGyroCommand;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem.Color;
import frc.robot.subsystems.Claw.ClawIOSparkMax;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final SendableChooser<String> m_autoSwitcher = new SendableChooser<String>();
  public static final String Forward = "Forward";
  public static final String ForwardRotate = "Forward + Rotate";
  public static final String NewPath = "New Path";
  // public String m_autoSelected;
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final ClawSubsystem m_claw = new ClawSubsystem(new ClawIOSparkMax());
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();  
  private final CANdleSubsystem m_ledSubsystem = new CANdleSubsystem();
  
  public static final double pi = Math.PI;
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandJoystick m_gunner = new CommandJoystick(1);

  public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // switch(Constants.currentMode){
    //   case REAL:
    //   m_drivetrainSubsystem = new DrivetrainSubsystem(new GyroIONavx(), 
    //     new SwerveModule(0, Constants.Drivetrain.Mod0.constants),
    //     new SwerveModule(1, Constants.Drivetrain.Mod1.constants), 
    //     new SwerveModule(2, Constants.Drivetrain.Mod2.constants), 
    //     new SwerveModule(3, Constants.Drivetrain.Mod3.constants));
    //   case SIM:
    //     m_drivetrainSubsystem = new DrivetrainSubsystem(new GyroIO(), 
    //       new SwerveModuleIOSim(), 
    //       new SwerveModuleIOSim(), 
    //       new SwerveModuleIOSim(), 
    //       new SwerveModuleIOSim());
    //   default:
    //     m_drivetrainSubsystem = new DrivetrainSubsystem(new GyroIO(), 
    //       new SwerveModule(0, Constants.Drivetrain.Mod0.constants),
    //       new SwerveModule(1, Constants.Drivetrain.Mod1.constants), 
    //       new SwerveModule(2, Constants.Drivetrain.Mod2.constants), 
    //       new SwerveModule(3, Constants.Drivetrain.Mod3.constants));
    //   }

    
    // m_autoSwitcher.addOption(Rotate, Rotate);
    m_autoSwitcher.setDefaultOption(NewPath, NewPath);
    m_autoSwitcher.addOption(Forward, Forward);
    m_autoSwitcher.addOption(ForwardRotate, ForwardRotate);
    
    // m_field = new Field2d();
    SmartDashboard.putData(m_autoSwitcher);
    
    
    configureButtonBindings();
      
  }

       
    
    //modifyAxis(value) no exponent

    // Configure the button bindings
    
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new SequentialCommandGroup( 
        new WaitCommand(1),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(new Pose2d()))
    ).schedule();
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(m_driverController.getLeftX()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getLeftY()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getRightX()/1.5) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            
    ));

    // m_claw.setDefaultCommand(new IntakeCommand(m_claw, Direction.CLOSE));

    m_driverController.y().onTrue(new SequentialCommandGroup(
      new TurnTo90(m_drivetrainSubsystem),
      new WaitCommand(1),
      new AlignToCone(m_drivetrainSubsystem, m_visionSubsystem)
    ));

    m_driverController.rightBumper().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));

    m_driverController.rightBumper().whileTrue(
      new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(m_driverController.getLeftX()/2.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getLeftY()/2.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getRightX()/2.5) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND     
    ));
    
    m_driverController.b().onTrue(new BalanceCommand(m_drivetrainSubsystem));
    
    m_driverController.a().toggleOnTrue(new MoveWithClosest90(
      m_drivetrainSubsystem, 
      () -> modifyAxis(m_driverController.getLeftX()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(-m_driverController.getLeftY()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
    ));

    m_gunner.button(2).onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));
    m_gunner.button(5).toggleOnFalse(new ToggleLEDCommand(m_ledSubsystem, new Color(242, 60, 0) ));
    m_gunner.button(5).toggleOnTrue(new ToggleLEDCommand(m_ledSubsystem, new Color(184, 0, 185)));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Middle Path", 2, 1);
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
    
    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();
    
    //1.0676
    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
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
      swerveTrajectoryFollower.withTimeout(15).andThen(new BalanceCommand(m_drivetrainSubsystem))
      );
  }

  }