package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetXConfigCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public SetXConfigCommand(DrivetrainSubsystem dt) {
        this.m_drivetrainSubsystem = dt;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.mSwerveMods[3].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 1/4), 0.0));
        m_drivetrainSubsystem.mSwerveMods[2].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 3/4), 0.0));

        m_drivetrainSubsystem.mSwerveMods[1].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 3/4), 0.0));
        m_drivetrainSubsystem.mSwerveMods[0].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 1/4), 0.0));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}