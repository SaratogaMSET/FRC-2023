package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;

public class ManualRunIntakeCommand extends CommandBase {
    
    RollerSubsystem roller;
    double speed =0;
    public ManualRunIntakeCommand(RollerSubsystem rollerSubsystem, double speed){
        this.roller = rollerSubsystem;
        this.speed = speed;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void execute(){
        roller.set_intake(speed);
    }

    @Override
    public void end(boolean interrupted){
        roller.set_intake(0);
    }

    @Override
    public boolean isFinished(){
        if(speed>0){
            return roller.objectInRoller();
        }
        return false;
    }
}
