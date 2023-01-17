package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sampleArmCommand;
import frc.robot.subsystems.ArmSubsystem;

public class sampleTestArmCommand extends SequentialCommandGroup{
    
    public sampleTestArmCommand(ArmSubsystem armSubsystem){

        try{
            addCommands(
                new sampleArmCommand(armSubsystem, 2,2)
            );
            SmartDashboard.putBoolean("Arm Working", true);
        }
        catch (Exception e) {
            SmartDashboard.putBoolean("Arm Working", false);
        }

    }
}
