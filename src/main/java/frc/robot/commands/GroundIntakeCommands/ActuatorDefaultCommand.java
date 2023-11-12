package frc.robot.commands.GroundIntakeCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ActuatorDefaultCommand extends CommandBase{
    ActuatorSubsystem actuator;
    public ActuatorDefaultCommand(ActuatorSubsystem actuatorSubsystem){
        actuator = actuatorSubsystem;
       
        addRequirements(actuatorSubsystem);
    }


    @Override
    public void execute(){
        actuator.idle();
        SmartDashboard.putBoolean("Actuator Angle running", false);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
