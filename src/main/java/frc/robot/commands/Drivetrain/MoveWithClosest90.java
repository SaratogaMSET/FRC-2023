package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveWithClosest90 extends CommandBase {

    double desiredAngle;
    private DrivetrainSubsystem drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    PIDController controller = new PIDController(2,0.0,0.0);
    double xT;
    double yT;
    int axis = 0;
    double lastRot;
    public MoveWithClosest90(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier){
        
        this.drivetrain = drivetrainSubsystem;
        this.x = translationXSupplier;
        this.y = translationYSupplier;
        addRequirements(drivetrainSubsystem);  

    }
    @Override
    public void initialize(){
        controller.enableContinuousInput(-180, 180);
        axis = (int) drivetrain.getRotation2d().getDegrees() / 90;
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

        xT = (x.getAsDouble()-xT)/5 + xT;
        yT = (y.getAsDouble()-yT)/5 + yT;

        double magnitude = Math.hypot(xT, yT);

        double joyAngle = Math.atan2(yT, xT);
        double roboAngle = (drivetrain.getNavHeading()+ joyAngle); 

        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;
        
        double pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;
        drivetrain.drive(
            new ChassisSpeeds(
                resultX,
                resultY,
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
        return false;
    }
}
