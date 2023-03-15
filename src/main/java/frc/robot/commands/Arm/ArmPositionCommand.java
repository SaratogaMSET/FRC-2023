package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmPositionCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private double tX;
    private double tY;
    private boolean hold = false;
    private double tolerance = 0.03;
    public ArmPositionCommand(ArmSubsystem armSubsystem, double tX, double tY){
        this.armSubsystem = armSubsystem;
        this.tX = tX;
        this.tY = tY;
        addRequirements(armSubsystem);
    } 
    public ArmPositionCommand(ArmSubsystem armSubsystem, double tX, double tY, double tolerance){
        this.armSubsystem = armSubsystem;
        this.tX = tX;
        this.tY = tY;
        this.tolerance = tolerance;
        addRequirements(armSubsystem);
    }  
    public ArmPositionCommand(ArmSubsystem armSubsystem, double tX, double tY, boolean hold){
        this.armSubsystem = armSubsystem;
        this.tX = tX;
        this.tY = tY;
        this.hold = hold;
        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        double[] iK = armSubsystem.inverseKinematics(tX, tY);
        armSubsystem.PIDtoAngles(iK[0], iK[1]);
        SmartDashboard.putNumber("Arm Position Command Error X", tX - armSubsystem.forwardKinematics()[0]);
        SmartDashboard.putNumber("Arm Position Command Error Y",tY - armSubsystem.forwardKinematics()[1]);
    }

    @Override
    public void end(boolean interrupted){
        // if(!interrupted)
        armSubsystem.voltageMotors(0, 0);
    }

    @Override
    public boolean isFinished(){
        double x_err = tX - armSubsystem.forwardKinematics()[0];
        double y_err = tY - armSubsystem.forwardKinematics()[1];
        if(hold){
            return false;
        }
        if(Math.abs(x_err) < tolerance && Math.abs(y_err) < tolerance){
            return true;
        }
        return false;
    }
}