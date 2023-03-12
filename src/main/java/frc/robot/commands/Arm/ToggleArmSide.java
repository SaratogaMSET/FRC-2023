package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ToggleArmSide extends CommandBase{
    ArmSubsystem m_armSubsystem;

    public ToggleArmSide(ArmSubsystem armSub){
        m_armSubsystem = armSub;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_armSubsystem.setSide();
    }

    public boolean isFinished(){
        return true;
    }
}
