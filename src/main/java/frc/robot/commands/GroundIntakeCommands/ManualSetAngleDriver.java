package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GroundIntake;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

public class ManualSetAngleDriver extends CommandBase {
    
    ActuatorSubsystem gIntakeSubsystem;
    double prevError;
    double errorDT;
    double error;
    double angle =0;
    boolean hold = false;
    double errorSum;
    public ManualSetAngleDriver(ActuatorSubsystem gIntakeSubsystem, double angle, boolean hold){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        this.hold = hold;
        addRequirements(gIntakeSubsystem);
    }

    public ManualSetAngleDriver(ActuatorSubsystem gIntakeSubsystem, double angle){
        this.gIntakeSubsystem = gIntakeSubsystem;
        this.angle = angle;
        addRequirements(gIntakeSubsystem);
    }

    @Override
    public void execute(){

        if(gIntakeSubsystem.getCurrent() > GroundIntake.currentLimit){
            gIntakeSubsystem.setVoltageActuator(0);
        }
        else{
            if(Math.abs(angle - gIntakeSubsystem.get_position_degrees()) > 3){
                errorDT = (error - prevError)/0.02;
                error = angle - gIntakeSubsystem.get_position_degrees();
                errorSum += error;
                gIntakeSubsystem.set_angle(angle, 100, errorSum, errorDT);
                SmartDashboard.putBoolean("Command running", true);
                prevError = error;
            }
            else{
                gIntakeSubsystem.tuneGravityCompensation();
            }
        }
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
