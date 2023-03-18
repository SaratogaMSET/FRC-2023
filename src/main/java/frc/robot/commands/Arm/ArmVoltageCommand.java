package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmVoltageCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private double pMotorVoltage;
    private double dMotorVoltage;
    private DoubleSupplier pMotorVoltageSupplier;
    private DoubleSupplier dMotorVoltageSupplier;
    public ArmVoltageCommand(ArmSubsystem armSubsystem, DoubleSupplier pMotorVoltageSupplier, DoubleSupplier dMotorVoltageSupplier){
        this.armSubsystem = armSubsystem;
        this.pMotorVoltageSupplier = pMotorVoltageSupplier;
        this.dMotorVoltageSupplier = dMotorVoltageSupplier;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
    //     if(Math.abs(pMotorVoltageSupplier.getAsDouble()-0.5) > 0.25 && Math.abs(dMotorVoltageSupplier.getAsDouble()-0.5) > 0.25)
    //      armSubsystem.voltageMotors(1, 1);
    //    else if(Math.abs(pMotorVoltageSupplier.getAsDouble()-0.5) > 0.25)
    //     armSubsystem.voltageMotors(1, 0);
    //     else if(Math.abs(dMotorVoltageSupplier.getAsDouble()-0.5) > 0.25)
    //     armSubsystem.voltageMotors(0, 1);
    //     else{ armSubsystem.voltageMotors(0, 0);} 
    armSubsystem.voltageMotors(0, 0);
    }  
     
    @Override
    public void end(boolean end){
        armSubsystem.voltageMotors(0, 0);
    }
      
}