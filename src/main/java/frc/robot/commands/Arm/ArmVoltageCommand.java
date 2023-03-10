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
        pMotorVoltage = pMotorVoltageSupplier.getAsDouble();
        dMotorVoltage = dMotorVoltageSupplier.getAsDouble();
        armSubsystem.voltageMotors(pMotorVoltage, dMotorVoltage);
    }         
}