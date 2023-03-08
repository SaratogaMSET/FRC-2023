package frc.robot.commands.Vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class AlignCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_vision;

    public AlignCommand(DrivetrainSubsystem m_drivetrainSubsystem, VisionSubsystem m_vision){
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        this.m_vision = m_vision;

        addRequirements(m_drivetrainSubsystem);
        addRequirements(m_vision);
    }
    @Override
    public void initialize(){}

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
