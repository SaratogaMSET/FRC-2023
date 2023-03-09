package frc.robot.commands.Auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AutoRunCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_velocityX;
    private double m_velocityY;
    private double m_DeltaTheta;
    private double m_initialAngle;
    private final PIDController pid;
    private double pidValue = 0;

    public AutoRunCommand(DrivetrainSubsystem drivetrainSubsystem, double velocityX, double velocityY, double deltaTheta) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_velocityX = velocityX;
        this.m_velocityY = velocityY;
        this.m_DeltaTheta = deltaTheta;
        

        pid = new PIDController(0.0,0.0,0.0); //0.1
        //+0.02
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.m_initialAngle = m_drivetrainSubsystem.getNavHeading();
    }

    @Override
    public void execute() {
        double currentAngle = m_drivetrainSubsystem.getNavHeading();
        double diff = currentAngle-m_initialAngle;
        pidValue = pid.calculate(diff, 0) * 50;

        SmartDashboard.putNumber("PID Value", pidValue);
        SmartDashboard.putNumber("Diff", diff * 180/3.1415);
        if(m_DeltaTheta == 0)
            m_drivetrainSubsystem.drive(new ChassisSpeeds(m_velocityX, m_velocityY, pidValue));
        else
            m_drivetrainSubsystem.drive(new ChassisSpeeds(m_velocityX, m_velocityY, m_DeltaTheta));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }
}
