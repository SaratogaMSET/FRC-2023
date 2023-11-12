package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;
public class RollerSubsystem extends SubsystemBase {
    //Current Limits as a failsafe
    SupplyCurrentLimitConfiguration IntakeLimit = new SupplyCurrentLimitConfiguration(
            true, 
            GroundIntake.intakeContinousCurrentLimit, 
            GroundIntake.intakePeakCurrentLimit, 
            0.1);
    
    //Constructor
    public RollerSubsystem(){
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configNeutralDeadband(0.00);
        motor.configSupplyCurrentLimit(IntakeLimit);
    }
    //Definition for motors, can id can be found in Constants.js
    WPI_TalonFX motor = new WPI_TalonFX(GroundIntake.intake_ID, "649-Hammerhead-CANivore");
    DigitalInput IRGate = new DigitalInput(GroundIntake.IR_GATE); //Infared Gate: Essentially a small laser. If an object intercepts the laser, this will return true. 

    public void setSpeed(double speed) {
        motor.set(speed); //motor.set(power) sets the motor power from a number between -1.0 and 1.0
    }
    public double getSpeed() {
        return motor.get();
    }
    public boolean objectInRoller() {
        return IRGate.get(); 
    }

}
