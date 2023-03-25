package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final PIDController pid = new PIDController(2, 0.0, 0.0);
    
    private Pose2d m_currentPose;
    private Pose2d m_targetPose;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem){
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        try {
            m_currentPose = m_drivetrainSubsystem.getPose();
            m_targetPose = Constants.Vision.ZONE_TO_TARGET_POSE.get(m_drivetrainSubsystem.getFieldZone());
            m_drivetrainSubsystem.drive(
                // it's so bad ahahahahahahahaha just kill me
                new ChassisSpeeds(
                    pid.calculate(m_currentPose.getX(), m_targetPose.getX()),
                    pid.calculate(m_currentPose.getY(), m_targetPose.getY()),
                    pid.calculate(m_currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians())
                )
            );
        } catch (Exception e) {
            System.out.println("Alignment failed. Exception: ");
            e.printStackTrace();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }

    @Override
    public boolean isFinished() {
        // TODO what're our actual margins of error?
        double translationMargin = 42;
        double rotationMargin = 42;
        var delta = m_targetPose.minus(m_currentPose);
        return MathUtils.isInRange(Math.abs(delta.getX()), -translationMargin, translationMargin) &&
            MathUtils.isInRange(Math.abs(delta.getY()), -translationMargin, translationMargin) &&
            MathUtils.isInRange(Math.abs(delta.getRotation().getRadians()), -rotationMargin, rotationMargin);
    }
}
