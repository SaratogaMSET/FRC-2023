package frc.robot.commands.Arm;

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
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;

public class ArmSequences{

    public static SequentialCommandGroup groundIntake(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y, 0.05);
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand closeIntake = new RunCommand(()-> m_clawSubsystem.manualCloseClaw());
        return ready.andThen(intake);
    }  
    
    public static ArmPositionCommand autonGroundIntake(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, ActuatorSubsystem actuator,int side){
        ArmPositionCommand intake;
        if(side > 0){
            intake = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.auton_intake_x), Constants.ArmNodeDictionary.auton_intake_y, 0.05);
            
            return intake;
        }else{
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.auton_intake_x , Constants.ArmNodeDictionary.auton_intake_y, 0.05);
            return intake;
        }
    }

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

    public static SequentialCommandGroup lowScore(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, int side){
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
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.openClaw());

        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }
    public static SequentialCommandGroup lowScoreNoRetract(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, int side){
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
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreCubeMid(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.openClaw());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreCubeMidNoRetract(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, 0.1, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y, 0.1, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreCubeHigh(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
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
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.openClaw());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
        
    }
    public static SequentialCommandGroup scoreCubeHighNoRetract(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
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
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeMid(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.openClaw());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeMidNoRetract(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y , true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());
        // return ready.andThen(score).andThen(openIntake).andThen(zero);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeHigh(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, int side){
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

    public static SequentialCommandGroup scoreConeHighNoRetract(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeHighNoRetractHighTolerance(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07, true);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        // ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score);
    }

    public static SequentialCommandGroup scoreConeHighNoRetractHighToleranceAuton(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y, 0.07);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen(new WaitCommand(0.15)).andThen(openIntake); //0.25
    }

    public static SequentialCommandGroup scoreCubeHighNoRetractHighTolerance(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07, true);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07, true);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score);
    }
    public static SequentialCommandGroup scoreCubeHighNoRetractHighToleranceAuton(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0) {
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07);
        } else {
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, 0.5);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y, 0.07);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        ParallelRaceGroup openIntake = new RunCommand(()-> m_clawSubsystem.openClaw()).until(()-> m_clawSubsystem.isClawFullyOpen());

        // return ready.andThen(score).andThen(openIntake);
        return ready.andThen(score).andThen(openIntake);
    }
    public static Command groundIntakeCone(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, int side) {
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ground_intake_cone_x, Constants.ArmNodeDictionary.ground_intake_cone_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_cone_x, Constants.ArmNodeDictionary.ground_intake_cone_y, true);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand closeIntake = new RunCommand(()-> m_clawSubsystem.manualCloseClaw());
        // return ready.andThen(intake).andThen(closeIntake).andThen(zero);
        return ready.andThen(intake);
    }

    public static Command groundIntakeTest(ArmSubsystem armSubsystem, ClawSubsystem m_clawSubsystem, ActuatorSubsystem actuator, RollerSubsystem rollers,int side) {
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.pick_up_ground_intake_x, Constants.ArmNodeDictionary.pick_up_ground_intake_y, true);
            
        }else{
            ArmPositionCommand intakeReady = new ArmPositionCommand(armSubsystem, ArmNodeDictionary.pick_up_ready_position_x, ArmNodeDictionary.pick_up_ready_position_y, 0.05);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.pick_up_ground_intake_x, Constants.ArmNodeDictionary.pick_up_ground_intake_y, true);
            ManualSetAngle dropIntake = new ManualSetAngle(actuator, 95);
            
            ManualRunIntakeCommand runRollers = new ManualRunIntakeCommand(rollers, 0.5 );

            // return (dropIntake.alongWith(runRollers).alongWith(new SequentialCommandGroup(intakeReady, intake))).until(()-> (m_clawSubsystem.isGamepieceInRange() && m_clawSubsystem.getGamePieceType() != null));
            return new SequentialCommandGroup(intakeReady, intake).until(()-> (m_clawSubsystem.isGamepieceInRange() && m_clawSubsystem.getGamePieceType() != null));
        }
        // return ready.andThen(intake).andThen(closeIntake).andThen(zero);
        return intake;
    }

    public static Command ZeroArm(ArmSubsystem armSubsystem){
        ArmPositionCommand intakeReady = new ArmPositionCommand(armSubsystem, ArmNodeDictionary.pick_up_ready_position_x, ArmNodeDictionary.pick_up_ready_position_y + 0.15, 0.1);
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        return intakeReady.andThen(zero);
    }
}