package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmSequences{

    public static SequentialCommandGroup groundIntake(ArmSubsystem armSubsystem, int side){
        ArmPositionCommand ready;
        ArmPositionCommand intake;
        if(side > 0){
            ready = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }else{
            ready = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ready_ground_intake_x, Constants.ArmNodeDictionary.ready_ground_intake_y);
            intake = new ArmPositionCommand(armSubsystem, -Constants.ArmNodeDictionary.ground_intake_x, Constants.ArmNodeDictionary.ground_intake_y);
        }
        ArmZeroCommand zero = new ArmZeroCommand(armSubsystem);

        return ready.andThen(intake).andThen(zero);
    }         
}