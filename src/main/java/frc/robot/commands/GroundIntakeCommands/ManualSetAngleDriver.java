package frc.robot.commands.GroundIntakeCommands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GroundIntake;
import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;


public class ManualSetAngleDriver extends CommandBase {
   
    ActuatorSubsystem IntakeSubsystem;
    double prevError;
    double errorDT;
    double error;
    double angle =0;
    boolean hold = false;
    double errorSum;
    public ManualSetAngleDriver(ActuatorSubsystem IntakeSubsystem, double angle, boolean hold){
        this.IntakeSubsystem = IntakeSubsystem;
        this.angle = angle;
        this.hold = hold;
        addRequirements(IntakeSubsystem);
    }


    public ManualSetAngleDriver(ActuatorSubsystem IntakeSubsystem, double angle){
        this.IntakeSubsystem = IntakeSubsystem;
        this.angle = angle;
        addRequirements(IntakeSubsystem);
    }


    @Override
    public void initialize(){
        errorSum = 0;
        errorDT = 0;
        error =0;
    }
    @Override
    public void execute(){


        // if(gIntakeSubsystem.getCurrent() > GroundIntake.currentLimit){
        //     gIntakeSubsystem.setVoltageActuator(0);
        // }
        // else{
            error = angle - IntakeSubsystem.getDegrees();
            errorDT = (error - prevError)/0.02;
            errorSum += error;
            IntakeSubsystem.setDegrees(angle, 100);
            SmartDashboard.putBoolean("Command running", true);
            prevError = error;
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
        // if(gIntakeSubsystem.getVoltage() < GroundIntake.voltageLimit){
        //     return true;
        // }
        return false;
    }
}


