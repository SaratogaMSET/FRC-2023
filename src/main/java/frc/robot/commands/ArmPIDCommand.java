package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private double angleP;
    private double angleD;
    public ArmPIDCommand(ArmSubsystem armSubsystem, double angleP, double angleD){
        this.armSubsystem = armSubsystem;
        this.angleP = angleP;
        this.angleD = angleD;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        armSubsystem.pidToAngles(angleP, angleD);
    }         
}
