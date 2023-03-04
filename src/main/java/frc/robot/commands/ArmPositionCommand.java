package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private double tX;
    private double tY;
    public ArmPositionCommand(ArmSubsystem armSubsystem, double tX, double tY){
        this.armSubsystem = armSubsystem;
        this.tX = tX;
        this.tY = tY;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        double[] iK = armSubsystem.inverseKinematics(tX, tY);
        armSubsystem.PIDtoAngles(iK[0], iK[1]);
        //armSubsystem.LQRtoAngles(iK[0], iK[1]);
    }         
}
