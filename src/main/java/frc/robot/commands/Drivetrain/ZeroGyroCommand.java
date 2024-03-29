package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class ZeroGyroCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    public ZeroGyroCommand(DrivetrainSubsystem dt) {
        this.m_drivetrainSubsystem = dt;
        addRequirements(m_drivetrainSubsystem);
    }
    public ZeroGyroCommand(DrivetrainSubsystem dt, String path){
        this.m_drivetrainSubsystem = dt;
    }
    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope();
        // if(path.equals(null)){
        // m_drivetrainSubsystem.zeroGyroscope();
        // }
        // else if(path.equals("Left")){
        //     m_drivetrainSubsystem.zeroGyroscopeAutonLeft();
        // }
        // else if(path.equals("Middle")){
        //     m_drivetrainSubsystem.zeroGyroscopeAutonMiddle();
        // }
        // else if(path.equals("Right")){
        //     m_drivetrainSubsystem.zeroGyroscopeAutonRight();
        // }
        // else{
        //     m_drivetrainSubsystem.zeroGyroscope();
        // }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}