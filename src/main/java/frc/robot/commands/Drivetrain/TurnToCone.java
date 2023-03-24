package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class TurnToCone extends CommandBase{

    private final DrivetrainSubsystem m_drivetrain; 
    private final VisionSubsystem m_visionSubsystem;
    PIDController controller = new PIDController(0.5, 0, 0.1);

    double pidValue;

    public TurnToCone(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem){
        this.m_drivetrain = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;
        controller.setTolerance(Math.PI/60); //0.4
        addRequirements(drivetrainSubsystem, visionSubsystem);  

    }
    @Override
    public void initialize(){
        controller.enableContinuousInput(-Math.PI, Math.PI);
        m_visionSubsystem.setPipeline(2);
    }

    @Override
    public void execute(){

        pidValue = controller.calculate(m_visionSubsystem.getOffsetTo2DOFBase()[2], 0); //-> to radians

        if (pidValue > 1) {
            pidValue = 0.5;
        } else if (pidValue < -1){
            pidValue = -0.5;
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

        m_drivetrain.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue) // in radians
        );
       
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrain.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }

    @Override
    public boolean isFinished(){
        // if(Math.abs(drivetrain.getRotation2d().getDegrees()-desiredAngle)<3) return true;
        if(Math.abs(controller.calculate(m_visionSubsystem.getOffsetTo2DOFBase()[2], 0)) < Math.PI/60) return true;
        else return false;
    }
}