package frc.robot.commands.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem.Color;

public class ToggleLEDCommand extends CommandBase {
    
    private final CANdleSubsystem candle;
    private final Color color;
    public ToggleLEDCommand(CANdleSubsystem ledSubsystem, Color color){
        this.candle = ledSubsystem;
        this.color = color;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute(){
        candle.setColor(color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
