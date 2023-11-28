// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.Arm.ArmSequences;
import frc.robot.commands.Arm.ArmVoltageCommand;
import frc.robot.commands.Arm.ArmZeroAutoCommand;
import frc.robot.commands.Arm.ArmZeroCommand;
import frc.robot.commands.Arm.ArmZeroStickyCommand;
import frc.robot.commands.Auton.AutoRunCommand;
import frc.robot.commands.Auton.AutonSequences;
import frc.robot.commands.CANdle.ManualStrobeCommand;
import frc.robot.commands.Drivetrain.BalanceCommand;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.MoveWithClosest90;
import frc.robot.commands.Drivetrain.TunableBalanceCommand;
import frc.robot.commands.Drivetrain.ZeroGyroCommand;
import frc.robot.commands.GroundIntakeCommands.ActuatorDefaultCommand;
import frc.robot.commands.GroundIntakeCommands.ManualRunIntakeCommand;
import frc.robot.commands.GroundIntakeCommands.ManualSetAngle;
import frc.robot.commands.GroundIntakeCommands.ManualSetAngleDriver;
import frc.robot.commands.WheelIntake.RunWheelExtakeCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.GyroIONavx;
import frc.robot.subsystems.Drivetrain.GyroIOSim;
import frc.robot.subsystems.Drivetrain.SwerveModuleIOFalcon;
import frc.robot.subsystems.Drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.WheelIntake.WheelIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // public static HashMap <String, Command> eventMap = new HashMap<>();
  public final SendableChooser<String> m_autoSwitcher = new SendableChooser<String>();
  public static final String OneAndBalance = "Middle One And Balance";
  public static final String OneAndBalanceBottom = "Bump Side One And Balance";
  public static final String TwoPieceTop = "Barrier Side Cone Preload and Cube score";
  public static final String OnePiece = " Anywhere but Middle: One Piece + Community Bonus";
  public static final String OnePlusHalf = "Bump Side 1.5 + Balance ";
  // public String m_autoSelected;  
  public static final String OneAndNothing = "One Score and NOTHING ELSE";
  // public static final String PhyscoBehavior = "Barrier Side Two Piece + Balance ";
  public static final String TwoPieceNoBalance = "Barrier Side 2 Piece No Balance";
  // public static final String ThreePiece = "Three Piece Test Path Barrier Side";
  public static final String BalanceMobilityBonus = "Middle Balance + Mobilty Bonus + Pickup";
  public static final String BalanceMobilityBonusNoPickup = "Middle Balance + Mobility NO PICKUP";
  public static final String TwoAndAHalfBalanceBarrier = "Barrier Side 2 Piece + Pickup";
  public static final String BottomTwoPiece = "Bump Side Two Piece";
  public static final String ChoreoTrajectory = "Choreo Trajectory";
  public static Boolean cone = false;
  
  public final static WheelIntake intake = new WheelIntake();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public static DrivetrainSubsystem m_drivetrainSubsystem = 
  new DrivetrainSubsystem(
      Robot.isReal()
          ? new SwerveModuleIOFalcon[] {
            new SwerveModuleIOFalcon(0, Constants.Drivetrain.Mod0.constants),
            new SwerveModuleIOFalcon(1, Constants.Drivetrain.Mod1.constants),
            new SwerveModuleIOFalcon(2, Constants.Drivetrain.Mod2.constants),
            new SwerveModuleIOFalcon(3, Constants.Drivetrain.Mod3.constants)
          }
          : new SwerveModuleIOSim[] {
            new SwerveModuleIOSim(0, Constants.Drivetrain.Mod0.constants),
            new SwerveModuleIOSim(1, Constants.Drivetrain.Mod1.constants),
            new SwerveModuleIOSim(2, Constants.Drivetrain.Mod2.constants),
            new SwerveModuleIOSim(3, Constants.Drivetrain.Mod3.constants)
          },
      Robot.isReal() ? new GyroIONavx() : new GyroIOSim(), 
      m_visionSubsystem::getEstimatedGlobalPose, 
      m_visionSubsystem::getTimestamp,
      m_visionSubsystem::getScaledSTDDevs);  
  private final CANdleSubsystem m_ledSubsystem = new CANdleSubsystem();
  private final ActuatorSubsystem actuatorSubsystem = new ActuatorSubsystem();
  private final RollerSubsystem rollers = new RollerSubsystem();

  public final static CommandXboxController m_driverController = new CommandXboxController(0);
  public final static CommandJoystick m_gunner1 = new CommandJoystick(1);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_autoSwitcher.setDefaultOption(OneAndNothing, OneAndNothing);
    m_autoSwitcher.addOption(OnePiece, OnePiece);
   
    m_autoSwitcher.addOption(OneAndBalanceBottom, OneAndBalanceBottom);
    m_autoSwitcher.addOption(OnePlusHalf, OnePlusHalf);
    m_autoSwitcher.addOption(BottomTwoPiece, BottomTwoPiece);
    
    m_autoSwitcher.addOption(OneAndBalance, OneAndBalance);
    m_autoSwitcher.addOption(BalanceMobilityBonusNoPickup, BalanceMobilityBonusNoPickup);
    m_autoSwitcher.addOption(BalanceMobilityBonus, BalanceMobilityBonus);

    m_autoSwitcher.addOption(TwoPieceNoBalance, TwoPieceNoBalance);
    m_autoSwitcher.addOption(TwoAndAHalfBalanceBarrier,TwoAndAHalfBalanceBarrier);
    
    // m_autoSwitcher.addOption(PhyscoBehavior, PhyscoBehavior);
    m_autoSwitcher.addOption(ChoreoTrajectory, ChoreoTrajectory);
    // m_autoSwitcher.addOption(ThreePiece, ThreePiece);
  

    // m_field = new Field2d();
    SmartDashboard.putData(m_autoSwitcher);
    
    
    configureButtonBindings();
      
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  // TODO: uncomment
    new SequentialCommandGroup( 
        new WaitCommand(1),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(new Pose2d()))
    ).schedule();
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(m_driverController.getLeftX() * 1.35) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, //1.2 or 2
            () -> modifyAxis(-m_driverController.getLeftY() * 1.35) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, //1.2 or 2
            () -> modifyAxis(-m_driverController.getRightX()/1.1) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> -modifyAxis(m_gunner1.getX(), 0.1) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_armSubsystem.getYPosition(),
            () -> actuatorSubsystem.get_position_degrees()
    ));

    m_armSubsystem.setDefaultCommand(
      new ArmVoltageCommand(
        m_armSubsystem
      ));

    m_ledSubsystem.setDefaultCommand(
      new ManualStrobeCommand(m_ledSubsystem)); // Change from StrobeCommand
    
    
    // m_gunner1.button(6).whileTrue(new ManualCloseIntake(intake));
    m_gunner1.button(6).whileTrue(intake.intakeCommand());
    m_gunner1.button(4).whileTrue(new RunWheelExtakeCommand(intake, 0.25));
    // m_gunner1.button(4).whileTrue(new RunCommand(()-> intake.openClaw(), intake));
    
    m_driverController.x().onTrue(new InstantCommand(()->m_drivetrainSubsystem.setX(), m_drivetrainSubsystem));

    m_driverController.y().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));
    m_driverController.leftTrigger().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));
    
    actuatorSubsystem.setDefaultCommand(new ActuatorDefaultCommand(actuatorSubsystem));
  
    
    m_driverController.b().toggleOnTrue(new SequentialCommandGroup(
      new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(m_driverController.getLeftX()/2.25) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getLeftY()/2.25) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getRightX()/2.25) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> modifyAxis(m_gunner1.getX(), 0.1) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> m_armSubsystem.getYPosition(),
            ()-> actuatorSubsystem.get_position_degrees()     
    )));

    // m_driverController.rightTrigger().onTrue(new AlignCommand(m_drivetrainSubsystem));

   
    m_driverController.a().toggleOnTrue(new MoveWithClosest90(
      m_drivetrainSubsystem, 
      () -> modifyAxis(m_driverController.getLeftX()* 1.35) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(-m_driverController.getLeftY()*1.35) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> m_armSubsystem.getYPosition(),
      () -> actuatorSubsystem.get_position_degrees(),
      () -> -modifyAxis(m_gunner1.getX(), 0.1) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
    ));

     m_driverController.rightBumper().onTrue(
      ArmSequences.ready(m_armSubsystem, 1).alongWith(intake.intakeCommand()) 
      );

    m_driverController.rightBumper().and(m_gunner1.button(1)).onTrue(
       ArmSequences.ready(m_armSubsystem, 1).alongWith(intake.intakeCommand())
       );
    
    m_driverController.leftBumper().onTrue(
    new ParallelCommandGroup(
      new ArmZeroCommand(m_armSubsystem),
      new SequentialCommandGroup(
        // new WaitCommand(0.3),
        new ParallelCommandGroup(
        new ManualRunIntakeCommand(rollers, 0),
        new ManualSetAngle(actuatorSubsystem, 10)
        )
      )
    )
    );

    // m_gunner1.button(3).onTrue(ArmSequences.score(m_armSubsystem, 1));

    m_gunner1.button(5).onTrue(ArmSequences.readyMoreForward(m_armSubsystem, 1).alongWith(intake.intakeCommand())); //TODO: make 0 when collision detection working
    m_gunner1.button(5).and(m_gunner1.button(1)).onTrue(ArmSequences.readyMoreForward(m_armSubsystem, 1).alongWith(intake.intakeCommand()));

    m_gunner1.button(7).onTrue(ArmSequences.scoreConeHighNoRetract(m_armSubsystem, 1)); // TODO: make 0 when collision detection working
    m_gunner1.button(7).and(m_gunner1.button(1)).onTrue(ArmSequences.scoreConeHighNoRetract(m_armSubsystem, 1)); 

    m_gunner1.button(8).onTrue(ArmSequences.armDunk(m_armSubsystem, intake, 1)); // TODO: make 0 when collision detection working
    m_gunner1.button(8).and(m_gunner1.button(1)).onTrue(ArmSequences.armDunk(m_armSubsystem, intake, 1));  

    // m_gunner1.button(9).onTrue(ArmSequences.scoreConeMidNoRetract(m_armSubsystem, 1)); // TODO: make 0 when collision detection working
    m_gunner1.button(9).onTrue(new ConditionalCommand(ArmSequences.scoreCubeMidNoRetract(m_armSubsystem, 1), ArmSequences.scoreConeMidNoRetract(m_armSubsystem, 1), ()->CANdleSubsystem.getIsCubeState())); // TODO: make 0 when collision detection working
    m_gunner1.button(9).and(m_gunner1.button(1)).onTrue(ArmSequences.scoreConeMidNoRetract(m_armSubsystem, 1));
    m_gunner1.button(10).onTrue(ArmSequences.armDunkMiddle(m_armSubsystem, intake, 1));// TODO: make 0 when collision detection working
    m_gunner1.button(10).and(m_gunner1.button(1)).onTrue(ArmSequences.armDunkMiddle(m_armSubsystem, intake, 1));

    // m_gunner1.button(11).toggleOnTrue(new MoveWithClosest90(
    // m_drivetrainSubsystem,
    // () -> modifyAxis(m_driverController.getLeftX() * 1.3) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, //1.2 or 2
    // () -> modifyAxis(-m_driverController.getLeftY() * 1.3) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, //1.2 or 2
    // () -> m_armSubsystem.getYPosition(),
    // () -> actuatorSubsystem.get_position_degrees(),
    // ()-> -modifyAxis(m_gunner1.getX(), 0.1) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND));
    // // m_gunner1.button(11).and(m_gunner1.button(1)).onTrue((ArmSequences.groundIntakeCone(m_armSubsystem, intake,  1)));



    m_gunner1.button(1)
      .whileTrue(
      new ParallelCommandGroup(new ManualSetAngleDriver(actuatorSubsystem, 100), new ManualRunIntakeCommand(rollers, 1))
      ) //.until( ()-> (intake.isGamepieceInRange() && intake.getGamePieceType() != null))))
      .onFalse(
        (new ArmZeroCommand(m_armSubsystem)).andThen(new ParallelCommandGroup(new ManualSetAngleDriver(actuatorSubsystem, 10), new ManualRunIntakeCommand(rollers, 0.0)))
      );

      m_gunner1.button(2).whileTrue(
        new ManualRunIntakeCommand(rollers, -0.25)) // -0.375
        .onFalse(new ManualRunIntakeCommand(rollers, 0.0));

      m_gunner1.button(12).whileTrue(
        new ManualRunIntakeCommand(rollers, 0.25, false)
      );
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

  private static double modifyAxis(double value, double deadband){
    value = deadband(value, deadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  // public void updateRobotState(){
  //   // RobotState.armSide = m_armSubsystem.getSide();
  //   // SmartDashboard.putNumber("side of the arm variety", m_armSubsystem.getSide());
  //   // SmartDashboard.putNumber("armside", RobotState.armSide);
  //   // SmartDashboard.putNumber("prox of the yimity", sensor.getProximity());
  //   // SmartDashboard.putBoolean("hall effect", HallEffect.get());
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selected = m_autoSwitcher.getSelected();
    switch(selected){
      case OnePiece:
        return AutonSequences.getOnePieceCommand(m_drivetrainSubsystem, m_armSubsystem, intake);
      case OneAndBalance:
        return AutonSequences.getOnePieceAndBalanceBringArmBackCommand(m_drivetrainSubsystem, m_armSubsystem, intake);
      case OneAndBalanceBottom:
        return AutonSequences.getOnePieceAndBalanceBottomCommand(m_drivetrainSubsystem, m_armSubsystem, intake);
      case OnePlusHalf:
        return AutonSequences.getBottomOneAndHalfPieceBalance(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case OneAndNothing:
        return AutonSequences.getOnePieceCommandOnly(m_drivetrainSubsystem, m_armSubsystem, intake);
      case BalanceMobilityBonusNoPickup:
        return AutonSequences.getOnePieceBalanceMobilityBonusNoPickup(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case BalanceMobilityBonus:
        return AutonSequences.getOnePieceBalanceMobilityBonus(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      // case PhyscoBehavior:
      //     return AutonSequences.getTwoPieceBalanceAutoBuilder(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case TwoPieceNoBalance:
        return AutonSequences.getTwoPieceNoBalance(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers ,intake);
      // case ThreePiece:
      //   return AutonSequences.getThreePieceAutoBuilder(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case TwoAndAHalfBalanceBarrier:
        return AutonSequences.getTwoAndAHalfPieceBalanceAutoBuilder(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case BottomTwoPiece:
        return AutonSequences.getBottomTwoPiece(m_drivetrainSubsystem, m_armSubsystem, actuatorSubsystem, rollers, intake);
      case ChoreoTrajectory:
        return AutonSequences.ChoreoCommand(m_drivetrainSubsystem); 
      default:
        return AutonSequences.getOnePieceCommandOnly(m_drivetrainSubsystem, m_armSubsystem, intake);
    }
  }

  }