package frc.robot.commands.CANdle;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem.Color;
import frc.robot.subsystems.Claw.ClawSubsystem;

@Deprecated
public class StrobeCommand extends CommandBase {
    
    private final CANdleSubsystem candle;
    private final ClawSubsystem m_claw;
    private double time = 0; 
    private boolean F = true;

    // private final Color color;
    public StrobeCommand(CANdleSubsystem ledSubsystem, ClawSubsystem claw){
        this.candle = ledSubsystem;
        this.m_claw = claw;
        addRequirements(ledSubsystem);
    }


    // public StrobeCommand(CANdleSubsystem ledSubsystem){
    //     this.candle = ledSubsystem;
    //     addRequirements(ledSubsystem);
    // }

    @Override
    public void execute(){
        if(!m_claw.hasAcquiredGamePiece()){
            double index = (70 - m_claw.encoder.getPosition())/(2);

            if(index > 35) index = 35;
            if(index < 9) index = 9;

            // candle.mapClawPosition((int) index);
        }
        else if (m_claw.hasAcquiredGamePiece()){
            // candle.strobe(new Color(56, 209, 0));
        }
        // candle.idle();
    }

    @Override
    public void end(boolean interrupted) {
    }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }
}