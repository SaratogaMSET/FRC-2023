package frc.robot.commands.WheelIntake;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelIntake.WheelIntake;

public class HoldAcquiredGamePiece extends CommandBase{
    WheelIntake intake;
    
    public HoldAcquiredGamePiece(WheelIntake wheelIntake){
        this.intake = wheelIntake;
        addRequirements(intake);
    }


    @Override
    public void initialize(){
        // intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 7, 10, 0.1));
    }
    @Override
    public void execute(){
        // if(actuator.getCurrent() > GroundIntake.currentLimit){
        //     actuator.setVoltageActuator(0);
        // }
        // else{
            intake.holdGamePiece();
        // }
    }

    @Override
    public void end(boolean interrupted){
        intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 2));
    }
}
