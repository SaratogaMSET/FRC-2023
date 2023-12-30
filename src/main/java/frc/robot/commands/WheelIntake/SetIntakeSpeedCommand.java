package frc.robot.commands.WheelIntake;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelIntake.WheelIntake;

public class SetIntakeSpeedCommand extends CommandBase {
    WheelIntake wheelIntake;
    double speed;
    public SetIntakeSpeedCommand(WheelIntake wheelIntake, double speed){
        this.wheelIntake = wheelIntake;
        addRequirements(wheelIntake);
        this.speed = speed;
    }

    @Override
    public void initialize(){
        wheelIntake.set(speed);
    }
    @Override
    public void execute(){
        wheelIntake.set(speed);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean interrupted){
        wheelIntake.set(0);
        wheelIntake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,15, 40, 2));
    }
}
