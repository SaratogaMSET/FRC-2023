package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

public class sampleArmCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private final double pvoltage;
    private final double dvoltage;

    public sampleArmCommand(ArmSubsystem armSubsystem, double pvoltage, double dvoltage){
        this.armSubsystem = armSubsystem;
        this.pvoltage = pvoltage;
        this.dvoltage = dvoltage;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        armSubsystem.voltageMotors(pvoltage, dvoltage);
    }
    
    @Override
    public void end(boolean interrupted){
        armSubsystem.voltageMotors(pvoltage/2, dvoltage/2);
        new WaitCommand(1);
        armSubsystem.voltageMotors(0, 0);
    }
}