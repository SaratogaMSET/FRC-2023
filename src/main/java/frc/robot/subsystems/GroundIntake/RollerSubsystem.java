package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;

public class RollerSubsystem extends SubsystemBase {
    WPI_TalonFX Intake = new WPI_TalonFX(GroundIntake.intake_ID, "649-Hammerhead-CANivore");
    DigitalInput gate = new DigitalInput(GroundIntake.IR_GATE);
    SupplyCurrentLimitConfiguration IntakeLimit = new SupplyCurrentLimitConfiguration(
            true, 
            GroundIntake.intakeContinousCurrentLimit, 
            GroundIntake.intakePeakCurrentLimit, 
            0.1);
    public RollerSubsystem(){
        Intake.setNeutralMode(NeutralMode.Brake);
        Intake.configNeutralDeadband(0.00);
        Intake.configSupplyCurrentLimit(IntakeLimit);
    }
    public void set_intake(double speed){
        Intake.set(speed);
    }

    public boolean objectInRoller(){
        return gate.get();  
    }
    public double getSpeed(){
        return Intake.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Is Object in Roller", objectInRoller());
    }
}
