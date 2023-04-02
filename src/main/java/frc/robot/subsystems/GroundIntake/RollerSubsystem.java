package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntake;

public class RollerSubsystem extends SubsystemBase {
    WPI_TalonFX Intake = new WPI_TalonFX(GroundIntake.intake_ID, "649-Hammerhead-CANivore");
    DigitalInput gate = new DigitalInput(GroundIntake.IR_GATE);
    
    public RollerSubsystem(){
        Intake.setNeutralMode(NeutralMode.Brake);
    }
    public void set_intake(double speed){
        Intake.set(speed);
    }

    public boolean objectInRoller(){
        return gate.get();  //true means it can see itself
    }
}
