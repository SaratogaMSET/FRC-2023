package frc.robot.commands.Arm;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Claw.ManualOpenIntake;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Claw.ClawIOSparkMax;

public class ArmSequences{

    public static SequentialCommandGroup groundIntake(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(intake).andThen(zero);
    }  
    
    public static SequentialCommandGroup autonGroundIntake(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand intake;
        if(side > 0){
            intake = new ArmPositionCommand(armSubsystem, -(Constants.ArmNodeDictionary.ground_intake_x + 0.1524), Constants.ArmNodeDictionary.ground_intake_y + (0.1524/2));
        }else{
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_x +  0.1524, Constants.ArmNodeDictionary.ground_intake_y + (0.1524/2));
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        return intake.andThen(zero);
    }

    public static SequentialCommandGroup groundIntakeNoRetract(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(intake).andThen(zero);
    }   

    // public static ArmPositionCommand ready(ArmSubsystem armSubsystem, IntSupplier side){
    //     ArmPositionCommand ready;
    //     if(side.getAsInt() > 0){
    //         ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
    //     }else{
    //         ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
    //     }

    //     return ready;
    // }
    public static ArmPositionCommand ready(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, true);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y, true);
        }

        return ready;
    }

    public static SequentialCommandGroup lowScore(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_low_score_x, Constants.ArmNodeDictionary.ready_low_score_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(score).andThen(zero);
    }

    public static SequentialCommandGroup scoreCubeMid(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcube_score_x, Constants.ArmNodeDictionary.ready_midcube_score_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(score).andThen(zero);
    }

    public static SequentialCommandGroup scoreCubeHigh(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(score).andThen(zero);
    }

    public static SequentialCommandGroup scoreCubeHighNoRetract(ArmSubsystem armSubsystem, ClawIOSparkMax m_clawSubsystem,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcube_score_x, Constants.ArmNodeDictionary.ready_highcube_score_y);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.autoCloseIntake());

        return ready.andThen(score).andThen(openIntake);
    }

    public static SequentialCommandGroup scoreConeMid(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_midcone_score_x, Constants.ArmNodeDictionary.ready_midcone_score_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(score).andThen(zero);
    }

    public static SequentialCommandGroup scoreConeHigh(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(score).andThen(zero);
    }

    public static SequentialCommandGroup scoreConeHighNoRetract(ArmSubsystem armSubsystem, ClawIOSparkMax m_clawSubsystem ,int side){
        ArmPositionCommand ready;
        ArmPositionCommand score;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_double_substation_x, Constants.ArmNodeDictionary.ready_double_substation_y);
            score = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_highcone_score_x, Constants.ArmNodeDictionary.ready_highcone_score_y);
        }
        // ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);
        RunCommand openIntake = new RunCommand(()-> m_clawSubsystem.autoCloseIntake());

        return ready.andThen(score).andThen(openIntake);
    }
}