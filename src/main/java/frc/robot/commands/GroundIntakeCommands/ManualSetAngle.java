package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GroundIntake;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ManualSetAngle extends CommandBase {
    
    ActuatorSubsystem IntakeSubsystem;
    double angle =0;
    boolean hold = false;
    public ManualSetAngle(ActuatorSubsystem IntakeSubsystem, double angle, boolean hold){
        this.IntakeSubsystem = IntakeSubsystem;
        this.angle = angle;
        this.hold = hold;
        addRequirements(IntakeSubsystem);
    }

    public ManualSetAngle(ActuatorSubsystem gIntakeSubsystem, double angle){
        this.IntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        addRequirements(IntakeSubsystem);
    }

    @Override
    public void execute(){
        // else{
            IntakeSubsystem.setDegrees(angle, 100);
            // SmartDashboard.putBoolean("Command running", true);
        // }
    }

    @Override
    public void end(boolean interrupted){
        // SmartDashboard.putBoolean("Command running", false);
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
