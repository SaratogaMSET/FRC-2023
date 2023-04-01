package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.IntakeSubsystem;

public class ManualRunIntakeCommand extends CommandBase {
    
    IntakeSubsystem gIntakeSubsystem;
    double speed =0;
    public ManualRunIntakeCommand(IntakeSubsystem gIntakeSubsystem, double speed){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.speed = speed;
        addRequirements(gIntakeSubsystem);
    }

    @Override
    public void execute(){
        gIntakeSubsystem.set_intake(speed);
    }

    @Override
    public void end(boolean interrupted){
        gIntakeSubsystem.set_intake(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
