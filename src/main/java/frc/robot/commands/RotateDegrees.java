package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RotateDegrees extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final PIDController pid;
    public double pidValue = 0;
    private double angle;
    private double initialAngle;
    double currentAngle;
    double diff;
    int count=0;
    // public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    // public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public RotateDegrees(DrivetrainSubsystem drivetrainSubsystem, double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = null;
        this.angle = angle;
        pid = new PIDController(Constants.Drivetrain.kPThetaController-0.06, 0, 0);
         initialAngle = m_drivetrainSubsystem.getNavHeading();
        addRequirements(m_drivetrainSubsystem);
    }

    public RotateDegrees(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSystem;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, 0, 0);

        addRequirements(m_drivetrainSubsystem);
        addRequirements(m_visionSubsystem);
    }




    @Override
    public void execute() {
        count++;
        // m_visionSubsystem.refresh();

        if(m_visionSubsystem!=null){
            /* if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) > 9) {
                pidValue = 9;
            } else if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) < -9) {
                pidValue = -9;
            } else {
                pidValue = pid.calculate(m_visionSubsystem.getRawAngle(), 0);
            }
            
            VisionSystem.VisionState visionState = m_visionSubsystem.updateVisionState();

            // failsafe code to make sure it does not keep spinning
            if(visionState==VisionSystem.VisionState.NO_TARGET){
                pidValue = 0;
            } */
        }
        else{
            
            currentAngle = m_drivetrainSubsystem.getNavHeading();
             diff = (initialAngle- currentAngle) * 180/Math.PI;
            pidValue = pid.calculate(diff, angle);
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,pidValue));
            SmartDashboard.putNumber("PID value", pidValue);

            SmartDashboard.putNumber("Diff", currentAngle*180/Math.PI);
        }

        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue
            )
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(m_visionSubsystem!=null){
        /*if (Math.abs(0 - m_visionSubsystem.getRawAngle()) < 3) {
            return true;
        }
        return false; */
    }
    else{
        // if(count>=50){
            if(Math.abs(angle-diff)< 10){
                return true;
            }

            else{
                return false;
            }
        // }
       
    //    return false;
    }

    return true;
    }
   
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}