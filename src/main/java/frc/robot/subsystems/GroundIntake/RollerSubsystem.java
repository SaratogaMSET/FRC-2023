package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
    WPI_TalonFX Intake = new WPI_TalonFX(Constants.GroundIntake.intake_ID, "649-Hammerhead-CANivore");

    
    public void set_intake(double speed){
        Intake.set(speed);
    }
}
