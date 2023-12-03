package frc.robot.commands.GroundIntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GroundIntake;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ActuatorDefaultCommand extends CommandBase{
    ActuatorSubsystem actuator;
    public ActuatorDefaultCommand(ActuatorSubsystem actuatorSubsystem){
        actuator = actuatorSubsystem;
       
        addRequirements(actuatorSubsystem);
    }


    @Override
    public void execute(){
        // if(actuator.getCurrent() > GroundIntake.currentLimit){
        //     actuator.setVoltageActuator(0);
        // }
        // else{
            actuator.stationary();
        // }
    }
}
