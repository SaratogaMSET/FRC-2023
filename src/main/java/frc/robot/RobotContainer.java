// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.Auton.AutoRunCommand;
import frc.robot.commands.Claw.BackUpIntakeCommand;
import frc.robot.commands.Claw.ManualCloseIntake;
import frc.robot.commands.Claw.ManualOpenIntake;
import frc.robot.commands.Drivetrain.AlignToCone;
import frc.robot.commands.Drivetrain.BalanceCommand;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.MoveWithClosest90;
import frc.robot.commands.Drivetrain.TunableBalanceCommand;
import frc.robot.commands.Drivetrain.TurnTo90;
import frc.robot.commands.Drivetrain.TurnToCone;
import frc.robot.commands.Drivetrain.ZeroGyroCommand;
import frc.robot.commands.Vision.AlignCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // public static HashMap <String, Command> eventMap = new HashMap<>();
  public final SendableChooser<String> m_autoSwitcher = new SendableChooser<String>();
  public static final String OneAndBalance = "One And Balance";
  public static final String OneAndBalanceBottom = "One And Balance Bottom";
  public static final String TwoPieceTop = "Cone Preload and Cube score Top";
  public static final String OnePiece = "One Piece and Community";
  public static final String OnePlusHalf = "Score, get One more and Balance Bottom";
  // public String m_autoSelected;  
  public static final String OneAndNothing = "One Score and Nothing";
  public final SendableChooser<Boolean> autoCloseChooser = new SendableChooser<Boolean>();
  public static final Boolean disableAutoClose = false;
  public static final Boolean enableAutoClose = true;

  
  public final ClawSubsystem m_claw = new ClawSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();  
  private final CANdleSubsystem m_ledSubsystem = new CANdleSubsystem();
  // private final ClawSubsystem m_clawSubsystem = new ClawSubsystem(new ClawIOSparkMax());
  
  public static final double pi = Math.PI;
  public final static CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandJoystick m_gunner1 = new CommandJoystick(1);
  private final CommandJoystick m_gunner2 = new CommandJoystick(2);

  public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // public HashMap<String, Command> eventMap = new HashMap<>(
  //   Map.ofEntries(
  //   Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetract(m_armSubsystem, m_claw, 1)),
  //   Map.entry("Arm Neutral", new ArmZeroCommand(m_armSubsystem)),
  //   Map.entry("Intake Front", ArmSequences.autonGroundIntake(m_armSubsystem, m_claw, 0)), 
  //   Map.entry("Arm Ready", ArmSequences.ready(m_armSubsystem, 1)),
  //   Map.entry("Score Cube High Backwards", ArmSequences.scoreCubeHighNoRetract(m_armSubsystem, m_claw,1)),
  //   Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem)),
  //   Map.entry("Balance", new BalanceCommand(m_drivetrainSubsystem))
  //   )
  //   );
  
  //1.0676
  // BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
  //   m_drivetrainSubsystem::getPose, 
  //   m_drivetrainSubsystem::resetOdometry, 
  //   new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
  //   new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
  //   new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
  //   m_drivetrainSubsystem::drive, 
  //   eventMap, 
  //   true,
  //   m_drivetrainSubsystem);

  List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Test Path", new PathConstraints(2, 3), new PathConstraints(2, 3), new PathConstraints(2, 3), new PathConstraints(2, 1));
  // public Command build = swerveAutoBuilder.fullAuto(trajectory);

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
    m_autoSwitcher.setDefaultOption(OneAndNothing, OneAndNothing);
    m_autoSwitcher.addOption(OneAndBalance, OneAndBalance);
    m_autoSwitcher.addOption(OnePiece, OnePiece);
    m_autoSwitcher.addOption(TwoPieceTop, TwoPieceTop);
    m_autoSwitcher.addOption(OneAndBalanceBottom, OneAndBalanceBottom);
    m_autoSwitcher.addOption(OnePlusHalf, OnePlusHalf);
    
    
    autoCloseChooser.setDefaultOption("Disable the Auto Close", disableAutoClose);
    autoCloseChooser.addOption("Don't Disable Auto Close", enableAutoClose);

    // m_field = new Field2d();
    SmartDashboard.putData(m_autoSwitcher);
    SmartDashboard.putData(autoCloseChooser);
    
    
    configureButtonBindings();
      
  }

       

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
            () -> modifyAxis(m_driverController.getLeftX() * 1.2) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getLeftY() * 1.2) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-m_driverController.getRightX()/1.5) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> m_armSubsystem.getYPosition()
    ));

    m_armSubsystem.setDefaultCommand(
      new ArmVoltageCommand(
        m_armSubsystem,
        () -> m_gunner1.getY(), 
        () -> m_gunner1.getX()
      ));

    m_claw.setDefaultCommand(new BackUpIntakeCommand(
      m_claw, 
      ()-> DriverStation.isAutonomous(),
      ()-> autoCloseChooser.getSelected()));

    m_driverController.rightTrigger().toggleOnTrue(new RunCommand(() -> m_claw.manualCloseClaw(), m_claw));
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_claw.openClaw(), m_claw));
    
    m_gunner1.button(6).whileTrue(new ManualCloseIntake(m_claw));
    m_gunner1.button(4).whileTrue(new RunCommand(() -> m_claw.openClaw(), m_claw));
    
    m_driverController.x().onTrue(new InstantCommand(()->m_drivetrainSubsystem.setX(), m_drivetrainSubsystem));

    // m_driverController.y().onTrue(new SequentialCommandGroup(
    //   new TurnTo90(m_drivetrainSubsystem),
    //   new WaitCommand(1),
    //   new AlignToCone(m_drivetrainSubsystem, m_visionSubsystem)
    // ));
    m_driverController.y().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));

    m_gunner1.button(2).onTrue(new AlignCommand(m_drivetrainSubsystem));

    // m_driverController.x().toggleOnTrue(
    //   new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> modifyAxis(m_driverController.getLeftX()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> modifyAxis(-m_driverController.getLeftY()/1.5) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> modifyAxis(-m_driverController.getRightX()/1.5) * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND     
    // ));
    
    m_driverController.b().onTrue(new SequentialCommandGroup(
      new TunableBalanceCommand(m_drivetrainSubsystem)));

    m_gunner1.button(1).whileTrue(m_ledSubsystem.indicateActiveSide());
    m_gunner1.button(3).toggleOnTrue(new ConditionalCommand(m_ledSubsystem.indicateConeCommand(), m_ledSubsystem.indicateCubeCommand(), () -> m_gunner1.button(3).getAsBoolean()));
    // m_gunner1.button(3).toggleOnFalse(m_ledSubsystem.indicateConeCommand());
    
    m_driverController.a().toggleOnTrue(new MoveWithClosest90(
      m_drivetrainSubsystem, 
      () -> modifyAxis(m_driverController.getLeftX()* 1.2) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(-m_driverController.getLeftY()*1.2) * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> m_armSubsystem.getYPosition()
    ));

     m_driverController.rightBumper().onTrue(//new ConditionalCommand(
      ArmSequences.ready(m_armSubsystem, 0) //,
      // new ArmZeroAutoCommand(m_armSubsystem), 
      // () -> m_driverController.rightBumper().getAsBoolean()
      ); //);
    // m_driverController.rightBumper().toggleOnFalse(new ArmZeroCommand(m_armSubsystem));

    m_driverController.rightBumper().and(m_gunner1.button(1)).onTrue(
      // new ConditionalCommand(
       ArmSequences.ready(m_armSubsystem, 1)//,
      //  new ArmZeroAutoCommand(m_armSubsystem), 
      //  () -> m_driverController.rightBumper().getAsBoolean()
       );//);
    m_driverController.leftBumper().onTrue(new ArmZeroAutoCommand(m_armSubsystem)); //neutral

    // m_gunner1.button(3).onTrue(new ToggleArmSide(m_armSubsystem));
    m_gunner1.button(5).onTrue(ArmSequences.readyMoreForward(m_armSubsystem, 0));
    m_gunner1.button(5).and(m_gunner1.button(1)).onTrue(ArmSequences.readyMoreForward(m_armSubsystem, 1));

    m_gunner1.button(7).onTrue(ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 0));
    m_gunner1.button(7).and(m_gunner1.button(1)).onTrue(ArmSequences.scoreConeHighNoRetract(m_armSubsystem, m_claw, 1)); 

    m_gunner1.button(8).onTrue(ArmSequences.scoreCubeHighNoRetractHighTolerance(m_armSubsystem,m_claw, 0));
    m_gunner1.button(8).and(m_gunner1.button(1)).onTrue(ArmSequences.scoreCubeHighNoRetract(m_armSubsystem, m_claw, 1)); 
    

    m_gunner1.button(9).onTrue(ArmSequences.scoreConeMidNoRetract(m_armSubsystem, m_claw, 0));
    m_gunner1.button(9).and(m_gunner1.button(1)).onTrue(ArmSequences.scoreConeMidNoRetract(m_armSubsystem, m_claw, 1));
    m_gunner1.button(10).onTrue(ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0));
    m_gunner1.button(10).and(m_gunner1.button(1)).onTrue(ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1));

    m_gunner1.button(11).onTrue(ArmSequences.groundIntakeCone(m_armSubsystem, m_claw,  0)); 
    m_gunner1.button(11).and(m_gunner1.button(1)).onTrue(ArmSequences.groundIntakeCone(m_armSubsystem, m_claw, 1));

    m_gunner1.button(12).onTrue(ArmSequences.groundIntake(m_armSubsystem, m_claw, 0)); 
    m_gunner1.button(12).and(m_gunner1.button(1)).onTrue(ArmSequences.groundIntake(m_armSubsystem, m_claw, 1));
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

  public void updateRobotState(){
    // RobotState.armSide = m_armSubsystem.getSide();
    // SmartDashboard.putNumber("side of the arm variety", m_armSubsystem.getSide());
    // SmartDashboard.putNumber("armside", RobotState.armSide);
    // SmartDashboard.putNumber("prox of the yimity", sensor.getProximity());
    // SmartDashboard.putBoolean("hall effect", HallEffect.get());
  }
  public Command getTwoPieceAndBalanceBottomCommand(){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Bottom Path Part 1", new PathConstraints(2, 1.5));
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Bottom Path Part 2", new PathConstraints(2, 3));
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();

    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory1, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
    
    PPSwerveControllerCommandA swerveTrajectoryFollower1 = new PPSwerveControllerCommandA(
      trajectory2, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.65),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ParallelCommandGroup(
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower
        ),
        ArmSequences.groundIntake(m_armSubsystem, m_claw, 0),
        new ManualCloseIntake(m_claw).withTimeout(0.5),
        swerveTrajectoryFollower1,
        // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
        new ParallelCommandGroup(
        // new ArmZeroCommand(m_armSubsystem),
        new SequentialCommandGroup(
          new BalanceCommand(m_drivetrainSubsystem),
          new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(0.9), //TODO: tune
          new InstantCommand(()-> m_drivetrainSubsystem.setX())
        )
        )
      )
    );

}

  public Command getOnePieceAndBalanceBottomCommand(){
      PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Bottom Path", new PathConstraints(2, 1.5));
      PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());
  
  
      PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
      PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
      PIDController thetaController = new PIDController(
            Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
      
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      xController.reset();
      yController.reset();
      thetaController.reset();
  
      PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
        trajectory1, 
        m_drivetrainSubsystem::getPose,
        Constants.Drivetrain.m_kinematics2,
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::drive,
        true,
        m_drivetrainSubsystem
      );
      
  
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
        new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
        new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
        // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
        // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
        new SequentialCommandGroup(
          ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
          new WaitCommand(0.65),
          // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
          // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
          new ManualOpenIntake(m_claw),
          // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
          new ParallelCommandGroup(
          new ArmZeroAutoCommand(m_armSubsystem),
          swerveTrajectoryFollower
          ),
          // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
          new ParallelCommandGroup(
          // new ArmZeroCommand(m_armSubsystem),
          new SequentialCommandGroup(
            new BalanceCommand(m_drivetrainSubsystem),
            new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(0.9), //TODO: tune 
            new InstantCommand(()-> m_drivetrainSubsystem.setX())
          )
          )
        )
      );

  }
  public Command getOnePieceAndBalanceCommand(){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Middle Path", 2, 1); // 2 3
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); 
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();

    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory1, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.65),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ParallelCommandGroup(
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower
        ),
        // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1),
        new ParallelCommandGroup(
        // new ArmZeroCommand(m_armSubsystem),
        new SequentialCommandGroup(
          new BalanceCommand(m_drivetrainSubsystem),
          new AutoRunCommand(m_drivetrainSubsystem, -((6 * 0.1524)/1.5), 0, 0).withTimeout(0.9), //TODO: tune 
          new InstantCommand(()-> m_drivetrainSubsystem.setX())
        )
        )
      )
    );
  }

  public Command getOnePieceAndBalanceBringArmBackCommand(){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Middle Path", 2, 1); // 2 3 
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();

    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory1, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.65),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ParallelCommandGroup(
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower
        ),
        ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1),
        new ParallelCommandGroup(
        new ArmZeroCommand(m_armSubsystem),
        new SequentialCommandGroup(
          new BalanceCommand(m_drivetrainSubsystem),
          new AutoRunCommand(m_drivetrainSubsystem, -((6 * 0.1524)/1.5), 0, 0).withTimeout(0.9), //TODO: tune 
          new InstantCommand(()-> m_drivetrainSubsystem.setX())
        )
        )
      )
    );
  }

  public Command getOnePieceCommand(){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("One Piece", 4, 1);
    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();

    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory1, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.65),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ParallelCommandGroup(
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower
        )
        // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
        // new ParallelCommandGroup(
        // new ArmZeroCommand(m_armSubsystem),
        // new SequentialCommandGroup(
        //   new BalanceCommand(m_drivetrainSubsystem),
        //   new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(1.1),
        //   new InstantCommand(()-> m_drivetrainSubsystem.setX())
        // )
        // )
      )
    );
  }
  public Command getOnePieceCommandOnly(){
    // PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("One Piece", 4, 1);
    // PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    // PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    // PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    // PIDController thetaController = new PIDController(
    //       Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // xController.reset();
    // yController.reset();
    // thetaController.reset();

    // PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
    //   trajectory1, 
    //   m_drivetrainSubsystem::getPose,
    //   Constants.Drivetrain.m_kinematics2,
    //   xController,
    //   yController,
    //   thetaController,
    //   m_drivetrainSubsystem::drive,
    //   true,
    //   m_drivetrainSubsystem
    // );
    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      // new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.65),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ParallelCommandGroup(
        new ArmZeroAutoCommand(m_armSubsystem)
        // swerveTrajectoryFollower
        )
        // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
        // new ParallelCommandGroup(
        // new ArmZeroCommand(m_armSubsystem),
        // new SequentialCommandGroup(
        //   new BalanceCommand(m_drivetrainSubsystem),
        //   new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(1.1),
        //   new InstantCommand(()-> m_drivetrainSubsystem.setX())
        // )
        // )
      )
    );
  }

  public Command getTwoPieceTopCommand(){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Top Path Part 1", 2, 3);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Top Path Part 2", 2, 3);

    PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory1.getInitialState(), DriverStation.getAlliance());


    PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME
    PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME
    PIDController thetaController = new PIDController(
          Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    thetaController.reset();

    PPSwerveControllerCommandA swerveTrajectoryFollower = new PPSwerveControllerCommandA(
      trajectory1, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );
    PPSwerveControllerCommandA swerveTrajectoryFollower1 = new PPSwerveControllerCommandA(
      trajectory2, 
      m_drivetrainSubsystem::getPose,
      Constants.Drivetrain.m_kinematics2,
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::drive,
      true,
      m_drivetrainSubsystem
    );

    

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(()-> m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0))),
      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation))),
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      // build.withTimeout(15).andThen(new InstantCommand(()->m_drivetrainSubsystem.setX()))
      new SequentialCommandGroup(
        ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1),
        new WaitCommand(0.5),
        // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
        // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
        new ManualOpenIntake(m_claw),
        // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower,
        ArmSequences.groundIntake(m_armSubsystem, m_claw, 0),
        new ArmZeroAutoCommand(m_armSubsystem),
        swerveTrajectoryFollower1,
        ArmSequences.scoreCubeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1)
        // new BalanceCommand(m_drivetrainSubsystem)
      )
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selected = m_autoSwitcher.getSelected();
    switch(selected){
      case OnePiece:
        return getOnePieceCommand();
      case OneAndBalance:
        return getOnePieceAndBalanceBringArmBackCommand();
      case OneAndBalanceBottom:
        return getOnePieceAndBalanceBottomCommand();
      case TwoPieceTop:
        getTwoPieceTopCommand();
      case OnePlusHalf:
        return getTwoPieceAndBalanceBottomCommand();
      case OneAndNothing:
        return getOnePieceCommandOnly();
      default:
        return getOnePieceCommand();
    }
  }

  }