package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Vision.PoseSmoothingFilter;

public class ZeroGyroCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private PoseSmoothingFilter m_poseSmoothingFilter;
    public ZeroGyroCommand(DrivetrainSubsystem dt, PoseSmoothingFilter m_poseSmoothingFilter) {
        this.m_drivetrainSubsystem = dt;
        this.m_poseSmoothingFilter = m_poseSmoothingFilter;

        addRequirements(m_drivetrainSubsystem, m_poseSmoothingFilter);
    }
    public ZeroGyroCommand(DrivetrainSubsystem dt, String path){
        this.m_drivetrainSubsystem = dt;
    }
    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope();
        m_poseSmoothingFilter.resetOdometry(m_poseSmoothingFilter.getPose());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}