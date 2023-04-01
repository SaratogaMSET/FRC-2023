package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ManualSetAngle extends CommandBase {
    
    ActuatorSubsystem gIntakeSubsystem;
    double angle =0;
    boolean hold = false;
    public ManualSetAngle(ActuatorSubsystem gIntakeSubsystem, double angle, boolean hold){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        this.hold = hold;
        addRequirements(gIntakeSubsystem);
    }

    public ManualSetAngle(ActuatorSubsystem gIntakeSubsystem, double angle){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        addRequirements(gIntakeSubsystem);
    }

    @Override
    public void execute(){
        gIntakeSubsystem.set_angle(angle, 100);
        SmartDashboard.putBoolean("Command running", true);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("Command running", false);
        // if(interrupted){
        //     new ManualSetAngle(gIntakeSubsystem, 10).schedule();
        // }
    }

    @Override
    public boolean isFinished(){
        // if(Math.abs(angle - gIntakeSubsystem.get_position_degrees()) < 3)
        // return true;
        return false;
    }
}
