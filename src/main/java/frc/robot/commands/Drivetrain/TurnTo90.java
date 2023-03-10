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
    PIDController controller = new PIDController(15,0.3,0.1);
    double xT;  
    double yT;
    int axis = 0;
    double lastRot;
    double pidValue;

    public TurnTo90(DrivetrainSubsystem drivetrainSubsystem){
        this.drivetrain = drivetrainSubsystem;
        controller.setTolerance(1);
        addRequirements(drivetrainSubsystem);  

    }
    @Override
    public void initialize(){
        controller.enableContinuousInput(-180, 180);
        this.axis = (int) drivetrain.getRotation2d().getDegrees() / 90; 
        if(drivetrain.getRotation2d().getDegrees() - (axis * 90) > 45) axis++ ;
        desiredAngle = axis * 90;
    }

    @Override
    public void execute(){
        lastRot = drivetrain.getRotation2d().getDegrees();
        /* axis */
        axis = ( (int) lastRot) / 90 -1;
        SmartDashboard.putNumber("Axis error", lastRot - axis * 90 );
        if(lastRot - axis * 90 > 45) axis++ ;
        desiredAngle = axis * 90; // desired angle is in degrees

        SmartDashboard.putNumber("desired angle", desiredAngle);

        pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180; //-> to radians

        if (pidValue > 1) {
            pidValue = 1;
        } else if (pidValue < -1){
            pidValue = -1;
        }

        SmartDashboard.putNumber("pidvalue", pidValue);

        /* turn to 0 */
        //desiredAngle = 0;

        /*  Pid testing shenanigans */

        // if (pidValue > 0) {
        //     pidValue = 0.3;
        // } else if (pidValue < 0){
        //     pidValue = 0.3;
        // }

        drivetrain.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue) // in radians
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
