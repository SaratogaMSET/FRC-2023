package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmNodeDictionary;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class MoveWithClosest90 extends CommandBase {

    double desiredAngle;
    private DrivetrainSubsystem drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier armHeight;
    DoubleSupplier actuatorHeight;
    PIDController controller = new PIDController(2,0.0,0.0);
    double xT;
    double yT;
    int axis = 0;
    double lastRot;
    double minArmValue = 0.3; 
    double maxArmValue = 1.18; 
    public MoveWithClosest90(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier armHeight,
        DoubleSupplier actuatorHeight){
        
        this.drivetrain = drivetrainSubsystem;
        this.x = translationXSupplier;
        this.y = translationYSupplier;
        this.armHeight = armHeight;
        this.actuatorHeight = actuatorHeight;
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
        // SmartDashboard.putNumber("Axis error", lastRot - axis * 90 );
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
        double armY = armHeight.getAsDouble();
        // double minArmHeight = 0.3;
        // double maxArmHeight = 1.18;
        // double iLerp = (armY - minArmHeight) /(minArmHeight - maxArmHeight);
        // double divider  = Math.max(0.0, Math.min(1.0, iLerp)) * 2.5;

        if(armY > Constants.ArmNodeDictionary.ready_midcube_score_y){      
            if(armHeight.getAsDouble() > minArmValue) {//start slow                 Constants.ArmNodeDictionary.ready_midcube_score_y){
                // double iLerp = (armY - minArmValue) / (minArmValue - maxArmValue); //inverse linear interpolation to get from 0 to 1 between min and max arm heights
                // double speedMultiplier = 1 + Math.max(0.0, Math.min(1.0, iLerp)) * (1.5); //clamp it actually between 0 to 1 in case oops! incident
                drivetrain.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                        resultX/2.5,
                        resultY/2.5,
                        pidValue)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
         }
        }
    else if(actuatorHeight.getAsDouble() > 50){
        drivetrain.drive(
            // ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
            resultX/1.5,
            resultY/1.5,
            pidValue)
            // m_drivetrainSubsystem.getRotation2d()
            // )
        );
            }
        else{
            drivetrain.drive(
                    // ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                    resultX,
                    resultY,
                    pidValue)
                    // m_drivetrainSubsystem.getRotation2d()
                    // )
            );
        }
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
