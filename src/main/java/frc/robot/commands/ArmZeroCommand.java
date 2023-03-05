package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmZeroCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    public ArmZeroCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        // double[] position = armSubsystem.forwardKinematics();
        // double minY = 0.05;
        // if(position[1] < minY){
        //     double[] armAngles = armSubsystem.Arm.inverseKinematics(position[0], minY + 0.03);
        //     armSubsystem.LQRtoAngles(armAngles[0], armAngles[1]);
        // }else{
        //     armSubsystem.LQRtoAngles(Math.PI/2, -Math.PI/2);
        // }
    }         
}
