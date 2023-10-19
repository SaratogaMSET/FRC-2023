package frc.robot.commands.WheelIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelIntake.WheelIntake;

public class RunWheelExtakeCommand extends CommandBase {
    WheelIntake wheelIntake;
    public RunWheelExtakeCommand(WheelIntake wheelIntake){
        this.wheelIntake = wheelIntake;
        addRequirements(wheelIntake);
    }
    @Override
    public void execute(){
        wheelIntake.set(-0.2);
    }

    @Override
    public void end(boolean interrupted){
        wheelIntake.set(0);
    }
}
