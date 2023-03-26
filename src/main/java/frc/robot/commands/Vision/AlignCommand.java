package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.DriveToPoseTrajectory;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private Pose2d m_currentPose;
    private Pose2d m_targetPose;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        try {
            m_currentPose = m_drivetrainSubsystem.getPose();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                m_targetPose = new Pose2d(Constants.Vision.RED_INITIAL_TARGET_POSE.getX(), Constants.Vision.RED_INITIAL_TARGET_POSE.getY() + Constants.Vision.Y_OFFSET_RED * (int) (m_currentPose.getY() / Constants.Vision.Y_OFFSET_RED), Constants.Vision.RED_INITIAL_TARGET_POSE.getRotation());
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                m_targetPose = new Pose2d(Constants.Vision.BLUE_INITIAL_TARGET_POSE.getX(), Constants.Vision.BLUE_INITIAL_TARGET_POSE.getY() +  Constants.Vision.Y_OFFSET_BLUE * (int) (m_currentPose.getY() / Constants.Vision.Y_OFFSET_BLUE), Constants.Vision.BLUE_INITIAL_TARGET_POSE.getRotation());
            }
            SmartDashboard.putNumberArray("Target pose", new double[]{m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getRadians()});
            new DriveToPoseTrajectory(m_drivetrainSubsystem, m_targetPose).schedule();
        } catch (Exception e) {
            System.out.println("Alignment failed. Exception: "); //TODO: Put an Alert on SmartDashboard and Shuffleboard
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
       
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        // TODO what're our actual margins of error?
        double translationMargin = 0.1;
        double rotationMargin = 0.1;
        var delta = m_targetPose.minus(m_currentPose);
        return MathUtils.isInRange(Math.abs(delta.getX()), -translationMargin, translationMargin) &&
                MathUtils.isInRange(Math.abs(delta.getY()), -translationMargin, translationMargin) &&
                MathUtils.isInRange(Math.abs(delta.getRotation().getRadians()), -rotationMargin, rotationMargin);
    }
}
