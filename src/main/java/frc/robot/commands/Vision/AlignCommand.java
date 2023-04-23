package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem.GamePiece;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class AlignCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ClawSubsystem clawSubsystem;
    private Pose2d m_currentPose;
    private Pose2d m_targetPose;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem, ClawSubsystem clawSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }
    @Override
    public void initialize() {
        try {
            m_currentPose = m_drivetrainSubsystem.getPose();
            
            if(DriverStation.getAlliance() == Alliance.Blue){
                if(clawSubsystem.getGamePieceType() == GamePiece.Cone){
                    m_targetPose = m_currentPose.nearest(Constants.Vision.BlueConeScoringPositions);
                }
                if(clawSubsystem.getGamePieceType() == GamePiece.Cube){
                    m_targetPose = m_currentPose.nearest(Constants.Vision.BlueCubeScoringPositions);
                }
            }

            if(DriverStation.getAlliance() == Alliance.Red){
                if(clawSubsystem.getGamePieceType() == GamePiece.Cone){
                    m_targetPose = m_currentPose.nearest(Constants.Vision.RedConeScoringPositions);    
                }
                if(clawSubsystem.getGamePieceType() == GamePiece.Cube){
                    m_targetPose = m_currentPose.nearest(Constants.Vision.RedCubeScoringPositions);
                }
            }
        }
        catch(Exception e){
            System.out.println(e + "cope harder");
        }
        try{
            SmartDashboard.putNumberArray("Target pose",
                    new double[] { m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getRadians() });
            SmartDashboard.putBoolean("Align command status: ", true);
            new DriveToPose(m_drivetrainSubsystem, m_targetPose).schedule();
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
