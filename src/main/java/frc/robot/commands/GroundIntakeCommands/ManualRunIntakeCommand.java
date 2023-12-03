package frc.robot.commands.GroundIntakeCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;
import frc.robot.subsystems.GroundIntake.RollerSubsystem;


public class ManualRunIntakeCommand extends CommandBase {
   
    RollerSubsystem roller;
    double speed =0;
    boolean useIRGate = true;
    public ManualRunIntakeCommand(RollerSubsystem rollerSubsystem, double speed){
        this.roller = rollerSubsystem;
        this.speed = speed;
        addRequirements(rollerSubsystem);
    }
    public ManualRunIntakeCommand(RollerSubsystem rollerSubsystem, double speed, boolean IRGate){
        this.roller = rollerSubsystem;
        this.speed = speed;
        this.useIRGate = IRGate;
        addRequirements(rollerSubsystem);
    }
    @Override
    public void execute(){
        roller.roll(speed);
    }


    @Override
    public void end(boolean interrupted){
        if(roller.acquired()){
            roller.roll(0.1); //arbitrary
        }
        else{
            roller.roll(0);
        }
    }


    @Override
    public boolean isFinished(){
        if(speed>0 && useIRGate){
            return roller.acquired();
        }
        return false;
    }
}


