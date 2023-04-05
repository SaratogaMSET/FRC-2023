package frc.robot.commands.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdle.CANdleSubsystem;

public class IndicateConeCommand extends CommandBase{
    CANdleSubsystem candle;
    public IndicateConeCommand(CANdleSubsystem caNdleSubsystem){
        this.candle = caNdleSubsystem;
    }
    public void initialize(){
        candle.indicateCone();
    }
    public boolean isFinished(){
        return true;
    }
}
