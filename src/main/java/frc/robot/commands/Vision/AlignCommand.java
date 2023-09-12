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
    /* This command is basically just to provide a framework for automating the alignment process before scoring
     * It doesn't do any moving, just calls the DriveToPose Command. 
     * It just choses the closest avaliable scoring node to go to. 
     * The biggest problem with this is the driver doesn't actually get any choice onto where they want to score. This is very untested 
     * and if it will ever be practical the driver needs to be able to choose where they want to score. 
    */

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ClawSubsystem clawSubsystem;

    private Pose2d m_currentPose;
    private Pose2d m_targetPose;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem, ClawSubsystem clawSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(m_drivetrainSubsystem); // we only need the status variables from the claw subsystem, don't need to move or anything
    }
    @Override
    public void initialize() {
        // Being careful about possible nulls thrown from clawSubsystem since it isn't required in the constructor. 
        try {
            m_currentPose = m_drivetrainSubsystem.getPose();
            
            // Selecting targets, pulling data from driver station and the claw sensors to figure out where we want to score. 
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

        // the actual movement command is in the DriveToPose command. Look there for more details. 
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


        // basically calculating if we're close enough to the scoring position to actually score.
        // The abs is unnecessary but MathUtils.isInRange just requires a min and max bound. 
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
