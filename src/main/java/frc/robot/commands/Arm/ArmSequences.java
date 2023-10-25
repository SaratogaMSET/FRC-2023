package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmNodeDictionary;
import frc.robot.commands.GroundIntakeCommands.ManualRunIntakeCommand;
import frc.robot.commands.GroundIntakeCommands.ManualSetAngle;
import frc.robot.commands.WheelIntake.RunWheelExtakeCommand;
import frc.robot.commands.WheelIntake.SetIntakeSpeedCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;
import frc.robot.subsystems.WheelIntake.WheelIntake;

public class ArmSequences{


    public static ArmPositionCommand ready(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, true);
        }
        return ready;
    }
    public static ArmPositionCommand readyMoreForward(ArmSubsystem armSubsystem ,int side){
        ArmPositionCommand ready;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_double_substation_x + ((0.125 + 0.09))/2), Constants.ArmNodeDictionary.ready_double_substation_y,true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x + ((0.125 + 0.09))/2, Constants.ArmNodeDictionary.ready_double_substation_y, true);
        }
        return ready;
    }

    public static SequentialCommandGroup lowScore(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }
    public static SequentialCommandGroup lowScoreNoRetract(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y, 0.25);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y, 0.1, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y, 0.25);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y, 0.1, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreCubeMid(ArmSubsystem armSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        // RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.openClaw());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreCubeHigh(ArmSubsystem armSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
        
    }
    public static SequentialCommandGroup scoreCubeHighNoRetract(ArmSubsystem armSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score);
    }

    //----COMMAND UNUSED LOL-----

    // public static SequentialCommandGroup scoreConeMid(ArmSubsystem armSubsystem,int side){
    //     ArmPositionCommand ready;
    //     ArmPositionCommand score;
    //     if(side > 0){
    //         ready = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_double_substation_x - 0.1), Constants.ArmNodeDictionary.ready_double_substation_y + 0.4, 0.5);
    //         score = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_midcone_score_x), Constants.ArmNodeDictionary.ready_midcone_score_y, true);
    //     }else{
    //         ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x - 0.1, Constants.ArmNodeDictionary.ready_double_substation_y + 0.4, 0.5);
    //         score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
    //     }
    //     ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
    //     // return ready.andThen(score).andThen(openIntake).andThen(zero);
    //     return ready.andThen(score);
    // }

    public static SequentialCommandGroup scoreCubeMidNoRetract(ArmSubsystem armSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommandMiddle score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_double_substation_x), Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.5);
            score = new ArmPositionCommandMiddle(armSubsystem, -(Constants.ArmNodeDictionary.ready_midcone_score_x) - 0.15, Constants.ArmNodeDictionary.ready_midcone_score_y - 0.1, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, (Constants.ArmNodeDictionary.ready_double_substation_x), Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.5);
            score = new ArmPositionCommandMiddle(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x - 0.15, Constants.ArmNodeDictionary.ready_midcone_score_y - 0.1, true);
        }
        // ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeMidNoRetract(ArmSubsystem armSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommandMiddle score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.5);
            score = new ArmPositionCommandMiddle(armSubsystem, -Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.5);
            score = new ArmPositionCommandMiddle(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
        }
        // ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeHigh(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;

        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
            
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.1);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
            
        }
        return ready.andThen(score);
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        
    }
    public static Command armDunkMiddle(ArmSubsystem armSubsystem, WheelIntake intake,  int side){
        // ArmPositionCommand ready;
        // ArmPositionCommand score;
        ArmPositionCommandMiddle dunk;
        ArmPositionCommand reset; 
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.03);
        if(side > 0) {
            // ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            // score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.3, false);
            dunk = new ArmPositionCommandMiddle(armSubsystem, -(Constants.ArmNodeDictionary.ready_midcone_score_x-0.06), Constants.ArmNodeDictionary.ready_midcone_score_y-0.35, 0.1, false);
            //reset = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_midcone_score_y, 0.05, false);
        } else {
            // ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            // score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.3, false);
            dunk = new ArmPositionCommandMiddle(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x-0.06, Constants.ArmNodeDictionary.ready_midcone_score_y-0.35, 0.1, false);
            //reset = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.05, false);
        }  
        
        // RunCommand openIntake = new RunCommand(()-> clawSubsystem.openClaw());
        return (dunk.andThen(new ArmZeroCommand(armSubsystem))).raceWith(extake).andThen(new SetIntakeSpeedCommand(intake, 0.0));
             //.andThen(reset).alongWith(new WaitCommand(0.3).andThen(openIntake)); //.andThen(new ArmZeroCommand(armSubsystem));

    }

    public static Command armDunk(ArmSubsystem armSubsystem, WheelIntake intake,int side){
        // ArmPositionCommand ready;
        // ArmPositionCommand score;
        ArmPositionCommand dunk;
        ArmPositionCommand reset; 
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.03);
        if(side > 0) {
            // ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            // score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.3, false);
            dunk = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_highcone_score_x +0.05), Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.1, false);
            reset = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.05, false);
        } else {
            // ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            // score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.3, false);
            dunk = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x + 0.05, Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.1, false);
            reset = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.05, false);
        }  
        
        // RunCommand openIntake = new RunCommand(()-> clawSubsystem.openClaw());
        return (((dunk).andThen((reset)).andThen(new ArmZeroCommand(armSubsystem)))).raceWith(extake).andThen(new SetIntakeSpeedCommand(intake, 0.0));//.andThen(reset).alongWith(new WaitCommand(0.3).andThen(openIntake)); //.andThen(new ArmZeroCommand(armSubsystem));

    }

    // public static Command scoreHighCommandGroup(ArmSubsystem armSubsystem, int side){
    //     if(clawSubsystem.getGamePieceType() == GamePiece.Cube){
    //         return scoreCubeHighNoRetract(armSubsystem, side);
    //     }
    //     else if(clawSubsystem.getGamePieceType() == GamePiece.Cone) {
    //         return scoreConeHighNoRetract(armSubsystem, side);
    //     }
    //     else{
    //         return ready(armSubsystem, side);
    //     }
    // }
    public static Command score(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand score;

        if(side > 0) {
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
    } else {
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        }
        return score;
    }

    public static Command scoreConeHighNoRetract(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        // if(side > 0) {
        //     if(Math.abs(armSubsystem.getYPosition() - (Constants.ArmNodeDictionary.ready_highcone_score_y-0.2)) < 0.1){
        //         ready =  new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        //         score = ready;
        //     }
        //     else{
        //         ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
        //         score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        //     }
        // } else {
        //     if(Math.abs(armSubsystem.getYPosition()  - (Constants.ArmNodeDictionary.ready_highcone_score_y-0.2)) < 0.1){
        //         ready =  new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        //         score = ready;
        //     }
        //     else{
        //         ready = new ArmPositionCommand(armSubsystem,Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
        //         score = new ArmPositionCommand(armSubsystem,Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        //     }
        // }
        if(side > 0) {
                ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
                score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        } else {
                ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
                score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
            }
        if(Math.abs(armSubsystem.getYPosition() - (Constants.ArmNodeDictionary.ready_highcone_score_y-0.2)) <= 0.6){
            return score;
           
        }
        else if(Math.abs(armSubsystem.getYPosition() - (Constants.ArmNodeDictionary.ready_highcone_score_y-0.2)) > 0.4){
            return ready.andThen(score);
        }else{
            return ready;
        }
    }

    public static SequentialCommandGroup scoreConeHighNoRetractHighTolerance(ArmSubsystem armSubsystem, WheelIntake intake, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.03);
        ArmPositionCommand dunk;
        ArmPositionCommand reset;

        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07, true);
            dunk = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_highcone_score_x +0.05), Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.3, true);
            reset = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.15, true);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07, true);
            dunk = new ArmPositionCommand(armSubsystem, (Constants.ArmNodeDictionary.ready_highcone_score_x +0.05), Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.3, true);
            reset = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.15, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        // ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen(dunk.alongWith(extake).andThen(reset));
    }
    
    public static SequentialCommandGroup scoreConeHighNoRetractHighToleranceAuton(ArmSubsystem armSubsystem, WheelIntake intake, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        ArmPositionCommand dunk;
        ArmPositionCommand reset;
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.03);
        
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07);
            dunk = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_highcone_score_x + 0.05), Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.1, false);
            reset = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ready_double_substation_x), Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.15, false);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07);
            dunk = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x + 0.05, Constants.ArmNodeDictionary.ready_highcone_score_y-0.2, 0.1, false);
            reset = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y + 0.25, 0.15, false);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        
        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen((dunk).andThen((reset).andThen(new ArmZeroAutoCommand(armSubsystem))).raceWith(extake)).andThen(new SetIntakeSpeedCommand(intake, 0.0));
    }

    public static SequentialCommandGroup scoreCubeHighNoRetractHighTolerance(ArmSubsystem armSubsystem, WheelIntake intake, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.2);
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07, true);

        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen(extake);
    }
    public static SequentialCommandGroup scoreCubeHighNoRetractHighToleranceAuton(ArmSubsystem armSubsystem, WheelIntake intake, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        RunWheelExtakeCommand extake = new RunWheelExtakeCommand(intake, 0.2);
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen(extake);
    }

    // public static Command groundIntakeTest(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers,int side) {
    //     ArmPositionCommand ready;
    //     ArmPositionCommand intake;
    //     if(side > 0){
    //         intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.pick_up_ground_intake_x, Constants.ArmNodeDictionary.pick_up_ground_intake_y, true);
            
    //     }else{
    //         ArmPositionCommand intakeReady = new ArmPositionCommand(armSubsystem, ArmNodeDictionary.pick_up_ready_position_x, ArmNodeDictionary.pick_up_ready_position_y, 0.05);
    //         intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.pick_up_ground_intake_x, Constants.ArmNodeDictionary.pick_up_ground_intake_y, true);
    //         ManualSetAngle dropIntake = new ManualSetAngle(actuator, 95);
            
    //         ManualRunIntakeCommand runRollers = new ManualRunIntakeCommand(rollers, 0.5 );

    //         // return (dropIntake.alongWith(runRollers).alongWith(new SequentialCommandGroup(intakeReady, intake))).until(()-> (m_clawSubsystem.isGamepieceInRange() && m_clawSubsystem.getGamePieceType() != null));
    //         return new SequentialCommandGroup(intakeReady, intake).until(()-> (m_clawSubsystem.isGamepieceInRange() && m_clawSubsystem.getGamePieceType() != null));
    //     }
        // return ready.andThen(intake).andThen(closeIntake).andThen(zero);
    //     return intake;
    // }

    public static Command ZeroArm(ArmSubsystem armSubsystem){
        ArmPositionCommand intakeReady = new ArmPositionCommand(armSubsystem, ArmNodeDictionary.pick_up_ready_position_x, ArmNodeDictionary.pick_up_ready_position_y + 0.15, 0.1);
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        return intakeReady.andThen(zero);
    }
}