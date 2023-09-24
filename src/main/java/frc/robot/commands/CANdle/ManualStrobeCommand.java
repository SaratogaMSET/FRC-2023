package frc.robot.commands.CANdle;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.CANdle.CANdleSubsystem.Color;
import frc.robot.subsystems.Claw.ClawSubsystem;

public class ManualStrobeCommand extends CommandBase {
    
    private final CANdleSubsystem candle;
    private final ClawSubsystem m_claw;
    private boolean previousGunnerValue = false;

    // private final Color color;
    public ManualStrobeCommand(CANdleSubsystem ledSubsystem, ClawSubsystem claw){
        this.candle = ledSubsystem;
        this.m_claw = claw;
        addRequirements(ledSubsystem);
    }

    private void toggleGunnerButton() {
        SmartDashboard.putBoolean("LED/Previous Gunner Boolean", previousGunnerValue);
        SmartDashboard.putBoolean("LED/Cone Boolean", RobotContainer.cone);
        if (!previousGunnerValue && RobotContainer.m_gunner1.button(4).getAsBoolean()){  // If previous gunner value is not the current value
            RobotContainer.cone = !RobotContainer.cone; // Set the current value to the new value (opposite of previous value)
        }
        previousGunnerValue = RobotContainer.m_gunner1.button(4).getAsBoolean();
    }
    // public StrobeCommand(CANdleSubsystem ledSubsystem){
    //     this.candle = ledSubsystem;
    //     addRequirements(ledSubsystem);
    // }
    @Override
    public void initialize(){
        // toggleGunnerButton();
        // if(m_claw.isGamepieceInRange()){
        //     if(RobotContainer.cone){
        //         candle.indicateConeFront();
        //     }
        //     else{
        //         candle.indicateCubeBack();
        //     }
        // }
        // else{
        //     if(RobotContainer.cone){
        //         candle.indicateConeFront();
        //     }
        //     else{
        //         candle.indicateCubeBack();
        //     }
        // }
    }


    @Override
    public void execute(){
        // if(!m_claw.hasAcquiredGamePiece()){
        //     double index = (70 - m_claw.encoder.getPosition())/(2);

        //     if(index > 35) index = 35;
        //     if(index < 9) index = 9;

        //     candle.mapClawPosition((int) index);
        // }
        // if (m_claw.hasAcquiredGamePiece()){
        //     candle.indicateConeBack();
        // }
        // else {
        //     m_claw.setFlash(false);
        //     candle.idle();
        // }


        // if(RobotContainer.flash){
        //     if(RobotContainer.cone){
        //         candle.indicateConeFront();
        //     }
        //     else{
        //         candle.indicateCubeFront();
        //     }
        // }
        // else{
        //     if(RobotContainer.cone){
        //         candle.indicateConeBack();
        //     }
        //     else{
        //         candle.indicateCubeBack();
        //     }
        // }

        toggleGunnerButton();
        if(m_claw.isGamepieceInRange()){
            if(RobotContainer.cone){
                candle.indicateConeFront();
            }
            else{
                candle.indicateCubeFront();
            }
        }
        else{
            if(RobotContainer.cone){
                candle.indicateConeBack();
            }
            else{
                candle.indicateCubeBack();
            }
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