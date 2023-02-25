package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.TrajectoryParser;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTrajectoryCommand extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private TrajectoryParser.TrajectoryData data;
    private double startTime = 0;
    public ArmTrajectoryCommand(ArmSubsystem armSubsystem, String name){
        this.armSubsystem = armSubsystem;
        TrajectoryParser parser = new TrajectoryParser();
        try{
            this.data = parser.parse(name);
        }catch(Exception e){
            this.data = null;
        }
        this.startTime = Timer.getFPGATimestamp();

        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        double time_since_start = Timer.getFPGATimestamp() - startTime;

        double[] states = data.stateAtTime(time_since_start);
        double[] controls = data.controlAtTime(time_since_start);

        SmartDashboard.putNumber("Traj ProximalQ", states[0]);
        SmartDashboard.putNumber("Traj DistalQ", states[1]);
        SmartDashboard.putNumber("Traj ProximalQdot", states[2]);
        SmartDashboard.putNumber("Traj DistalQdot", states[3]);
        SmartDashboard.putNumber("Traj ProximalU", controls[0]);
        SmartDashboard.putNumber("Traj DistalU", controls[1]);
        //double[] iK = armSubsystem.inverseKinematics(tX, tY);
        //armSubsystem.LQRtoAngles(iK[0], iK[1]);
    }         
}
