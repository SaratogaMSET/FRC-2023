package frc.robot.commands.GroundIntakeCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ManualSetAngle extends CommandBase {
    
    ActuatorSubsystem gIntakeSubsystem;
    double angle =0;

    public ManualSetAngle(ActuatorSubsystem gIntakeSubsystem, double angle){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        addRequirements(gIntakeSubsystem);
    }

    @Override
    public void execute(){
        // else{
            SmartDashboard.putBoolean("Actuator Angle running", true);
            gIntakeSubsystem.setAngle(angle, 100);
            
        // }
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
