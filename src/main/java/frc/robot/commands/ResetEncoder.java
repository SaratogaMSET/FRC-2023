package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetEncoder extends CommandBase{
    private DrivetrainSubsystem m_Dt;

    public ResetEncoder(DrivetrainSubsystem x){
        m_Dt = x;
    }

    @Override
    public void execute() {
        m_Dt.resetOdometry(new Pose2d());
    }

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
