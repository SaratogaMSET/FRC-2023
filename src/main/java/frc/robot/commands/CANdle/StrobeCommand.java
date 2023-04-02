package frc.robot.commands.CANdle;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem.Color;
import frc.robot.subsystems.Claw.ClawSubsystem;

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

        // if (m_led)
        if (m_claw.isGamepieceAcquired() && m_claw.getFlash()){
            candle.strobe();
            // try {
            //     wait(2000);
            // } catch (Exception e) {
            //     // TODO Auto-generated catch block
            //     e.printStackTrace();
        }
        else {
            m_claw.setFlash(false);
            candle.idle();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }
}