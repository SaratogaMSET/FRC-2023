package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Vision;

public class AlignToCone extends CommandBase{
    private final PIDController xController; // yController, angController;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    private double xVel;

    public AlignToCone(DrivetrainSubsystem m_drivetrainSubsystem, VisionSubsystem m_visionSubsystem){
        xController = new PIDController(Drivetrain.kPXController, 0.0, 0.0);
        //yController = new PIDController(Drivetrain.kPYController, 0.0, 0.0);
        //angController = new PIDController(Drivetrain.kPThetaController, 0.0, 0.0);    


        xController.setTolerance(0.4);
        //yController.setTolerance(0.4);
        //angController.setTolerance(0.4);
     
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        this.m_visionSubsystem = m_visionSubsystem;

        addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(-m_visionSubsystem.getOffsetTo2DOFBase()[0]); 
    }

    @Override
    public void execute() {
        this.xVel = xController.calculate(-m_visionSubsystem.getOffsetTo2DOFBase()[0]);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(-xVel, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds());
    }
}
