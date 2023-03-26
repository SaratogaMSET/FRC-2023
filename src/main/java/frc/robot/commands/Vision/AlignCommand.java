package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final PIDController pid = new PIDController(2, 0.0, 0.0);

    private Pose2d m_currentPose;
    private Pose2d m_targetPose;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        try {
            m_currentPose = m_drivetrainSubsystem.getPose();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                m_targetPose = new Pose2d(Constants.Vision.RED_INITIAL_TARGET_POSE.getX(), Constants.Vision.RED_INITIAL_TARGET_POSE.getY() + Constants.Vision.Y_OFFSET_RED * (int) (m_currentPose.getY() / Constants.Vision.Y_OFFSET_RED), Constants.Vision.RED_INITIAL_TARGET_POSE.getRotation());
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                m_targetPose = new Pose2d(Constants.Vision.BLUE_INITIAL_TARGET_POSE.getX(), Constants.Vision.BLUE_INITIAL_TARGET_POSE.getY() + -Constants.Vision.Y_OFFSET_BLUE * (int) (m_currentPose.getY() / Constants.Vision.Y_OFFSET_BLUE), Constants.Vision.BLUE_INITIAL_TARGET_POSE.getRotation());
            }
            SmartDashboard.putNumberArray("Target pose", new double[]{m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getRadians()});
            m_drivetrainSubsystem.drive(
                    // it's so bad ahahahahahahahaha just kill me
                    new ChassisSpeeds(
                            pid.calculate(m_currentPose.getX(), m_targetPose.getX()) / 2,
                            pid.calculate(m_currentPose.getY(), m_targetPose.getY()) / 2,
                            pid.calculate(m_currentPose.getRotation().getRadians(),
                                    m_targetPose.getRotation().getRadians()) / 2));
        } catch (Exception e) {
            System.out.println("Alignment failed. Exception: ");
            e.printStackTrace();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        // TODO what're our actual margins of error?
        double translationMargin = 0.2;
        double rotationMargin = 0.2;
        var delta = m_targetPose.minus(m_currentPose);
        return MathUtils.isInRange(Math.abs(delta.getX()), -translationMargin, translationMargin) &&
                MathUtils.isInRange(Math.abs(delta.getY()), -translationMargin, translationMargin) &&
                MathUtils.isInRange(Math.abs(delta.getRotation().getRadians()), -rotationMargin, rotationMargin);
    }
}
