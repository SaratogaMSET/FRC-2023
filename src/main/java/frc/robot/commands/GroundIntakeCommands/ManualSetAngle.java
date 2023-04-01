package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.IntakeSubsystem;

public class ManualSetAngle extends CommandBase {
    
    IntakeSubsystem gIntakeSubsystem;
    double angle =0;
    public ManualSetAngle(IntakeSubsystem gIntakeSubsystem, double angle){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        addRequirements(gIntakeSubsystem);
    }

    @Override
    public void execute(){
        gIntakeSubsystem.set_angle(angle);
        SmartDashboard.putBoolean("Command running", true);
    }

    @Override
    public void end(boolean interrupted){
        gIntakeSubsystem.setVoltageActuator(0.0);
        SmartDashboard.putBoolean("Command running", false);
    }

    @Override
    public boolean isFinished(){
        // if(Math.abs(angle - gIntakeSubsystem.get_position_degrees()) < 6)
        // return true;
        return false;
    }
}
