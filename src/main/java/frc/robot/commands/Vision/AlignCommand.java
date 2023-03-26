package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
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
            m_targetPose = new Pose2d(m_currentPose.getX(),
                    Constants.Vision.INITIAL_TARGET_POSE.getY()
                            + Constants.Vision.Y_OFFSET * (int) ((m_currentPose.getY()
                                    - Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL)
                                    / Constants.Vision.Y_OFFSET),
                    Constants.Vision.BLUE_INITIAL_TARGET_POSE.getRotation());
            SmartDashboard.putNumberArray("Target pose",
                    new double[] { m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getRadians() });
            SmartDashboard.putBoolean("Align command status: ", true);
            new DriveToPoseTrajectory(m_drivetrainSubsystem, m_targetPose).schedule();
        } catch (Exception e) {
            SmartDashboard.putBoolean("Align command status: ", false);
            System.out.println("Alignment failed. Exception: ");
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

        var delta = m_targetPose.minus(m_currentPose);
        return MathUtils.isInRange(Math.abs(delta.getX()), -Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL,
                Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL) &&
                MathUtils.isInRange(Math.abs(delta.getY()), -Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL,
                        Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL)
                &&
                MathUtils.isInRange(Math.abs(delta.getRotation().getRadians()),
                        -Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL,
                        Constants.Vision.ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL);
    }
}
