package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase{
    DrivetrainSubsystem drivetrain;
    double intitalPitch;
    public BalanceCommand(DrivetrainSubsystem m_drivetrainSubsystem){
        this.drivetrain = m_drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        intitalPitch = drivetrain.m_navx.getRoll();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }
}