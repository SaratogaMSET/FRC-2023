package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmZeroCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    public ArmZeroCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        double[] position = armSubsystem.forwardKinematics();
        double minY = 0.057 + 0.02;

        double groundintake_minY = minY + 0.2;
        // if(position[0] > 0 && position[1] < groundintake_minY){
        //     double[] armAngles = armSubsystem.inverseKinematics(position[0], groundintake_minY + 0.1);
        //     armSubsystem.PIDtoAngles(armAngles[0], armAngles[1]);
        // }
        if(position[1] < minY){
            double[] armAngles = armSubsystem.inverseKinematics(position[0], minY + 0.2);
            armSubsystem.PIDtoAngles(armAngles[0], armAngles[1]);
        }else{
            armSubsystem.PIDtoAngles(Math.PI/2, -Math.PI/2);
        }
    } 
            
    @Override
    public void end(boolean end){
        armSubsystem.voltageMotors(0, 0);
    }

    @Override
    public boolean isFinished(){
        double prox_err = armSubsystem.armInterface.getPositionProximal() - Math.PI/2;
        double dist_err = armSubsystem.armInterface.getPositionDistal() + Math.PI/2;
        double tolerance = 0.03;
        if(Math.abs(prox_err) < tolerance && Math.abs(dist_err) < tolerance){
            return true;
        }
        return false;
    }
}