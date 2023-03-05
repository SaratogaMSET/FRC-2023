package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmCartesianVelocity extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private DoubleSupplier vX;
    private DoubleSupplier vY;

    private double initialX;

    public ArmCartesianVelocity(ArmSubsystem armSubsystem, DoubleSupplier vXSupplier, DoubleSupplier vYSupplier){
        this.armSubsystem = armSubsystem;
        this.vX = vXSupplier;
        this.vY = vYSupplier;

        // this.initialX = armSubsystem.forwardKinematics()[0];

        addRequirements(armSubsystem);
    }   

    @Override
    public void execute(){
        // if(initialX * armSubsystem.forwardKinematics()[0] > 0 || initialX * vX.getAsDouble() > 0){
        //     armSubsystem.VelocityCartesian(vX.getAsDouble(), vY.getAsDouble());
        //     SmartDashboard.putString("CartVel Stat", "Moving");
        // }else{
        //     SmartDashboard.putString("CartVel Stat", "Frozen");
        //     SmartDashboard.putString("CartVel Check", "" + initialX + "," + armSubsystem.forwardKinematics()[0]);
        //     armSubsystem.voltageMotors(0, 0);
        // }
        
    }
}
