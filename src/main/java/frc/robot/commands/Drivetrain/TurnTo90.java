package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class TurnTo90 extends CommandBase {

    double desiredAngle;
    private DrivetrainSubsystem drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    PIDController controller = new PIDController(17.5,0.0,0.0);
    double xT;
    double yT;
    int axis = 0;
    double lastRot;
    double pidValue;

    public TurnTo90(DrivetrainSubsystem drivetrainSubsystem){
        
        this.drivetrain = drivetrainSubsystem;
        controller.setTolerance(4);
        addRequirements(drivetrainSubsystem);  

    }
    @Override
    public void initialize(){
        controller.enableContinuousInput(-180, 180);
        this.axis = (int) drivetrain.getRotation2d().getDegrees() / 90;
        if(drivetrain.getRotation2d().getDegrees() - axis * 90 > 45) axis++ ;
        desiredAngle = axis * 90;
    }

    @Override
    public void execute(){
        lastRot = drivetrain.getRotation2d().getDegrees();
        axis = ( (int) lastRot) / 90 -1;
        SmartDashboard.putNumber("Axis error", lastRot - axis * 90 );
        if(lastRot - axis * 90 > 45) axis++ ;
        desiredAngle = axis * 90;

        pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;
        drivetrain.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue)
        );
       
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }

    @Override
    public boolean isFinished(){
        // if(Math.abs(drivetrain.getRotation2d().getDegrees()-desiredAngle)<3) return true;
        if(pidValue == 0) return true;
        else return false;
    }
}
