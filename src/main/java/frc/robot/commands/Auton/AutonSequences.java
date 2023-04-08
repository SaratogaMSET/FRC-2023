package frc.robot.commands.Auton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.BetterSwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.Arm.ArmSequences;
import frc.robot.commands.Arm.ArmZeroAutoCommand;
import frc.robot.commands.Arm.ArmZeroCommand;
import frc.robot.commands.Claw.ManualCloseIntake;
import frc.robot.commands.Claw.ManualOpenIntake;
import frc.robot.commands.Drivetrain.BalanceCommand;
import frc.robot.commands.Drivetrain.FastBalanceCommand;
import frc.robot.commands.Drivetrain.TunableBalanceCommand;
import frc.robot.commands.Drivetrain.ZeroGyroCommand;
import frc.robot.commands.GroundIntakeCommands.ManualRunIntakeCommand;
import frc.robot.commands.GroundIntakeCommands.ManualSetAngle;
import frc.robot.commands.GroundIntakeCommands.ManualSetAngleDriver;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;

public class AutonSequences {

    public static SequentialCommandGroup getTwoPieceAndBalanceBottomCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            // ArmSequences.groundIntake(m_armSubsystem, m_claw, 0),
            new ManualCloseIntake(m_claw).withTimeout(0.5),
            swerveTrajectoryFollower1,
            // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
            new ParallelCommandGroup(
            // new ArmZeroCommand(m_armSubsystem),
            new SequentialCommandGroup(
              new BalanceCommand(m_drivetrainSubsystem),
              new AutoRunCommand(m_drivetrainSubsystem, Drivetrain.balanceXVelocity, 0, 0).withTimeout(Drivetrain.balanceTimeout), //TODO: tune
              new InstantCommand(()-> m_drivetrainSubsystem.setX())
            )
            )
          )
        );
    
    }
    
      public static SequentialCommandGroup getOnePieceAndBalanceBottomCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
              ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1),
              // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
              // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
              // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
              new ParallelCommandGroup(
              new ArmZeroCommand(m_armSubsystem),
              swerveTrajectoryFollower
              ),
              // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0),
              new ParallelCommandGroup(
              // new ArmZeroCommand(m_armSubsystem),
              new SequentialCommandGroup(
                new TunableBalanceCommand(m_drivetrainSubsystem),
                new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, (Drivetrain.balanceXVelocity), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout), //TODO: tune 
                new InstantCommand(()-> m_drivetrainSubsystem.setX())
              )
              )
            )
          );
    
      }
      public static SequentialCommandGroup getOnePieceAndBalanceCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1),
            // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
            // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
            // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
            new ParallelCommandGroup(
            new ArmZeroAutoCommand(m_armSubsystem),
            swerveTrajectoryFollower
            ),
            // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1),
            new ParallelCommandGroup(
            // new ArmZeroCommand(m_armSubsystem),
            new SequentialCommandGroup(
              new TunableBalanceCommand(m_drivetrainSubsystem),
              new AutoRunCommand(m_drivetrainSubsystem, -(Drivetrain.balanceXVelocity), 0, 0).withTimeout(1), //TODO: tune 
              new InstantCommand(()-> m_drivetrainSubsystem.setX())
            )
            )
          )
        );
      }
    
      public static SequentialCommandGroup getOnePieceAndBalanceBringArmBackCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1),
            // new WaitCommand(0.65),
            // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
            // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
            // new ManualOpenIntake(m_claw),
            // new InstantCommand(()->SmartDashboard.putBoolean("Claw Opened", false)),
            new ParallelCommandGroup(
            new ArmZeroCommand(m_armSubsystem),
            swerveTrajectoryFollower
            ),
            // ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1),
            new ParallelCommandGroup(
            new ArmZeroCommand(m_armSubsystem),
            new SequentialCommandGroup(
              new BalanceCommand(m_drivetrainSubsystem),
              new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, -((Drivetrain.balanceXVelocity)), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout), //TODO: tune 
              new InstantCommand(()-> m_drivetrainSubsystem.setX())
            )
            )
          )
        );
      }

      public static Command getOnePieceBalanceMobilityBonus(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuatorSubsystem, RollerSubsystem rollers,ClawSubsystem m_claw){

        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          // Map.entry("Arm Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero Command", new ArmZeroAutoCommand(m_armSubsystem)), 
          Map.entry("Intake Front", new ManualSetAngle(actuatorSubsystem, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngle(actuatorSubsystem, 10)),
          Map.entry("Zero Rollers", new ManualRunIntakeCommand(rollers, 0)),
          // Map.entry("Arm Extend Low", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0)),
          Map.entry("Balance Command", new FastBalanceCommand(m_drivetrainSubsystem))
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem))
          )
          );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Middle Path Builder", new PathConstraints(1.25, 1.25));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds((6 * 0.1524)/1.5, 0, 0, m_drivetrainSubsystem.getRotation2d());
        return build.andThen(new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, ((Drivetrain.balanceXVelocity +0.25)), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout + 0.15)).andThen(new InstantCommand(()-> m_drivetrainSubsystem.setX()));

      }

      public static Command getOnePieceBalanceMobilityBonusNoPickup(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuatorSubsystem, RollerSubsystem rollers,ClawSubsystem m_claw){

        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          // Map.entry("Arm Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero Command", new ArmZeroAutoCommand(m_armSubsystem)), 
          Map.entry("Intake Front", new ManualSetAngle(actuatorSubsystem, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngle(actuatorSubsystem, 10)),
          Map.entry("Zero Rollers", new ManualRunIntakeCommand(rollers, 0)),
          // Map.entry("Arm Extend Low", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0)),
          Map.entry("Balance Command", new TunableBalanceCommand(m_drivetrainSubsystem))
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem))
          )
          );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Middle Path Builder No Pickup", new PathConstraints(1.25, 1.25));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds((6 * 0.1524)/1.5, 0, 0, m_drivetrainSubsystem.getRotation2d());
        return (build).andThen(new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, ((Drivetrain.balanceXVelocity)), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout)).andThen(new InstantCommand(()-> m_drivetrainSubsystem.setX()));

      }
    
      public static Command getBottomOneAndHalfPieceBalance(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuatorSubsystem, RollerSubsystem rollers, ClawSubsystem m_claw){

        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          // Map.entry("Arm Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero Command", new ArmZeroCommand(m_armSubsystem)), 
          Map.entry("Intake Front", new ManualSetAngle(actuatorSubsystem, 95)),
          Map.entry("Rollers Run", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Intake Zero", new ManualSetAngle(actuatorSubsystem, 10)),
          Map.entry("Zero Rollers", new ManualRunIntakeCommand(rollers, 0)),
          // Map.entry("Arm Extend Low", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0)),
          Map.entry("Balance Command", new TunableBalanceCommand(m_drivetrainSubsystem))
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem))
          )
          );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Bottom Balance Pickup", new PathConstraints(2, 1), new PathConstraints(3, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build.andThen(new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, ((Drivetrain.balanceXVelocity)), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout)).andThen(new InstantCommand(()-> m_drivetrainSubsystem.setX()));

      }

      public static Command getBottomTwoPiece(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuatorSubsystem, RollerSubsystem rollers, ClawSubsystem m_claw){

        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          // Map.entry("Arm Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero Command", new ArmZeroCommand(m_armSubsystem)), 
          Map.entry("Intake Front", new ManualSetAngle(actuatorSubsystem, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngle(actuatorSubsystem, 10)),
          Map.entry("Zero Roller Speed", new ManualRunIntakeCommand(rollers, 0)),
          Map.entry("Extake Cube", new ManualRunIntakeCommand(rollers, -1)),
          // Map.entry("Arm Extend Low", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0)),
          Map.entry("Balance Command", new TunableBalanceCommand(m_drivetrainSubsystem))
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem))
          )
          );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Bottom 2 Piece", new PathConstraints(2, 1), new PathConstraints(3, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds((6 * 0.1524)/1.5, 0, 0, m_drivetrainSubsystem.getRotation2d());
        return build;

      }

      public static SequentialCommandGroup getOnePieceCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1),
            // new InstantCommand(()->SmartDashboard.putBoolean("Arm Scoring", true)),
            // new RunCommand(()->m_claw.openIntake(), m_claw).withTimeout(0.5)
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

      public static Command getThreePieceAutoBuilder(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers, ClawSubsystem m_claw){
        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Neutral", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Intake Front", new ManualSetAngle(actuator, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngle(actuator, 10)),
          Map.entry("Zero Roller Speed", new ManualRunIntakeCommand(rollers, 0.0)),
          Map.entry("Score Low", new ManualRunIntakeCommand(rollers, -0.7)),
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem)),
          // Map.entry("Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Front Intake", new ManualSetAngle(actuator, 95)),
          Map.entry("Rollers Run", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Roll Intake Up", new ManualSetAngle(actuator, 10)),
          Map.entry("Extake Cube", new ManualRunIntakeCommand(rollers, -1))
          )
       );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Top Path 3 Piece", new PathConstraints(4, 3), new PathConstraints(2, 1.5), new PathConstraints(2, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build; //.andThen(new BalanceCommand(m_drivetrainSubsystem)); //.andThen(new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(0.35));
      }

      public static Command getTwoPieceNoBalance(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers, ClawSubsystem m_claw){
        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Neutral", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Intake Front", new ManualSetAngleDriver(actuator, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngleDriver(actuator, 10)),
          Map.entry("Zero Roller Speed", new ManualRunIntakeCommand(rollers, 0.0)),
          Map.entry("Score Low", new ManualRunIntakeCommand(rollers, -0.7)),
          Map.entry("Arm is Neutral", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Down Intake", new ManualSetAngleDriver(actuator, 95)),
          Map.entry("Running Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Intake Up", new ManualSetAngleDriver(actuator, 10)),
          Map.entry("Stop Rollers",  new ManualRunIntakeCommand(rollers, 0.0) )
          )
       );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Top Path No Balance", new PathConstraints(4, 3), new PathConstraints(2, 1.5), new PathConstraints(2, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build; //.andThen(new BalanceCommand(m_drivetrainSubsystem)).andThen(new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(0.35));
      }

      public static Command getTwoPieceBalanceAutoBuilder(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers, ClawSubsystem m_claw){
        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Neutral", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Intake Front", new ManualSetAngleDriver(actuator, 95)),
          Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
          Map.entry("Zero Intake", new ManualSetAngleDriver(actuator, 10)),
          Map.entry("Zero Roller Speed", new ManualRunIntakeCommand(rollers, 0.0)),
          Map.entry("Score Low", new ManualRunIntakeCommand(rollers, -1)),
          Map.entry("Roller Zero", new ManualRunIntakeCommand(rollers, 0)),
          // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem)),
          // Map.entry("Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Balance", new FastBalanceCommand(m_drivetrainSubsystem))
          )
       );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Top Path", new PathConstraints(4, 3), new PathConstraints(2, 1.5), new PathConstraints(2.5, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build.andThen(new AutoRunCommand(m_drivetrainSubsystem, ChassisSpeeds.fromFieldRelativeSpeeds(0, -((Drivetrain.balanceXVelocity)), 0, m_drivetrainSubsystem.getRotation2d())).withTimeout(Drivetrain.balanceTimeout)).andThen(new InstantCommand(()-> m_drivetrainSubsystem.setX()));
      }

      public static Command getTwoAndAHalfPieceBalanceAutoBuilder(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers, ClawSubsystem m_claw){
        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
            Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
            Map.entry("Arm Zero Command", new ArmZeroCommand(m_armSubsystem)),
            Map.entry("Intake Front", new ManualSetAngleDriver(actuator, 95)),
            Map.entry("Run Rollers", new ManualRunIntakeCommand(rollers, 0.7)),
            Map.entry("Zero Intake", new ManualSetAngleDriver(actuator, 10)),
            Map.entry("Zero Roller Speed", new ManualRunIntakeCommand(rollers, 0.0)),
            Map.entry("Score Low", new ManualRunIntakeCommand(rollers, -1)),
            Map.entry("Roller Zero", new ManualRunIntakeCommand(rollers, 0)),
            Map.entry("Front Intake", new ManualSetAngleDriver(actuator, 95)),
            Map.entry("Rollers Run", new ManualRunIntakeCommand(rollers, 0.7)),
            Map.entry("Intake Up", new ManualSetAngleDriver(actuator, 95)),
            Map.entry("Zero Rollers", new ManualRunIntakeCommand(rollers, -0.7)),
            // Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem)),
            // Map.entry("Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
            Map.entry("Arm Zero", new ArmZeroCommand(m_armSubsystem)),
            Map.entry("Balance", new FastBalanceCommand(m_drivetrainSubsystem))
          )
       );
        
        // 1.0676
        BetterSwerveAutoBuilder swerveAutoBuilder = new BetterSwerveAutoBuilder(
          m_drivetrainSubsystem::getPose, 
          m_drivetrainSubsystem::resetOdometry, 
          new PIDConstants(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0), 
          new PIDConstants(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0),
          new PIDConstants(Constants.Drivetrain.kPThetaControllerTrajectory, 0, Constants.Drivetrain.kDThetaControllerTrajectory),
          m_drivetrainSubsystem::drive, 
          eventMap, 
          true,
          m_drivetrainSubsystem);

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Top Path 2.5 Piece Balance", new PathConstraints(2.75, 4));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build.andThen(new AutoRunCommand(m_drivetrainSubsystem, Drivetrain.balanceXVelocity, 0, 0).withTimeout(Drivetrain.balanceTimeout)).andThen(new InstantCommand(()-> m_drivetrainSubsystem.setX()));
      }

      public static SequentialCommandGroup getOnePieceCommandOnly(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1),
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
    
      public static SequentialCommandGroup getTwoPieceTopCommand(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
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
            // ArmSequences.groundIntake(m_armSubsystem, m_claw, 0),
            new ArmZeroAutoCommand(m_armSubsystem),
            swerveTrajectoryFollower1,
            ArmSequences.scoreCubeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1)
            // new BalanceCommand(m_drivetrainSubsystem)
          )
        );
      }
}
