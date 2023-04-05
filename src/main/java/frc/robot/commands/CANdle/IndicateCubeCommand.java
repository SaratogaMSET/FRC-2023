package frc.robot.commands.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdle.CANdleSubsystem;

public class IndicateCubeCommand extends CommandBase{
    CANdleSubsystem candle;
    public IndicateCubeCommand(CANdleSubsystem caNdleSubsystem){
        this.candle = caNdleSubsystem;
    }
    public void initialize(){
        candle.indicateCube();
    }
    public boolean isFinished(){
        return true;
    }
}