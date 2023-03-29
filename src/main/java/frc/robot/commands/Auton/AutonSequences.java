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
import frc.robot.commands.Arm.ArmSequences;
import frc.robot.commands.Arm.ArmZeroAutoCommand;
import frc.robot.commands.Arm.ArmZeroCommand;
import frc.robot.commands.Claw.ManualCloseIntake;
import frc.robot.commands.Claw.ManualOpenIntake;
import frc.robot.commands.Drivetrain.BalanceCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

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

      public static SequentialCommandGroup getOnePieceBalanceMobilityBonus(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){

        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero Command", new ArmZeroAutoCommand(m_armSubsystem)), 
          Map.entry("Arm Extend Low", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 0)),
          Map.entry("Balance Command", new BalanceCommand(m_drivetrainSubsystem)),
          Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem))
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

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Middle Path Builder", new PathConstraints(2, 1));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build.andThen(new AutoRunCommand(m_drivetrainSubsystem, (6 * 0.1524)/1.5, 0, 0).withTimeout(0.35));

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

      public static Command getTwoPieceBalanceAutoBuilder(DrivetrainSubsystem m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, ClawSubsystem m_claw){
        final HashMap<String, Command> eventMap = new HashMap<>(
          Map.ofEntries(
          Map.entry("Score Cone High Backwards", ArmSequences.scoreConeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Neutral", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Intake Front", ArmSequences.autonGroundIntake(m_armSubsystem, m_claw, 0)), 
          Map.entry("Close Claw", new RunCommand( ()-> m_claw.autoCloseClaw(), m_claw)),
          Map.entry("Zero the Arm", new ArmZeroAutoCommand(m_armSubsystem)),
          Map.entry("Arm Ready", ArmSequences.ready(m_armSubsystem, 1)),
          Map.entry("Score Cube High Backwards", ArmSequences.scoreCubeHighNoRetractHighToleranceAuton(m_armSubsystem, m_claw,1)),
          Map.entry("Arm Neutral Command", new ArmZeroCommand(m_armSubsystem)),
          Map.entry("Low Score Backwards", ArmSequences.lowScoreNoRetract(m_armSubsystem, m_claw, 1)),
          Map.entry("Arm Zero", new ArmZeroAutoCommand(m_armSubsystem)),
          Map.entry("Balance", new BalanceCommand(m_drivetrainSubsystem))
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

        List <PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Top Path", new PathConstraints(4, 3), new PathConstraints(2, 1.5), new PathConstraints(2, 3));
        Command build = swerveAutoBuilder.fullAuto(trajectory);
        return build;
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
            ArmSequences.groundIntake(m_armSubsystem, m_claw, 0),
            new ArmZeroAutoCommand(m_armSubsystem),
            swerveTrajectoryFollower1,
            ArmSequences.scoreCubeHighNoRetractHighTolerance(m_armSubsystem, m_claw, 1)
            // new BalanceCommand(m_drivetrainSubsystem)
          )
        );
      }
}
