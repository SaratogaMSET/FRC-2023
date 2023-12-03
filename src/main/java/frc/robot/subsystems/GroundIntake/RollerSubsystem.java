package frc.robot.subsystems.GroundIntake;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;




import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;


public class RollerSubsystem extends SubsystemBase {
    WPI_TalonFX roller = new WPI_TalonFX(GroundIntake.intake_ID, "649-Hammerhead-CANivore");
    DigitalInput sensor = new DigitalInput(GroundIntake.IR_GATE);
    SupplyCurrentLimitConfiguration IntakeLimit = new SupplyCurrentLimitConfiguration(
            true,
            GroundIntake.intakeContinousCurrentLimit,
            GroundIntake.intakePeakCurrentLimit,
            0.1);


    public RollerSubsystem(){
        roller.configSupplyCurrentLimit(IntakeLimit);
        roller.setNeutralMode(NeutralMode.Brake);
    }
    public void roll(double speed){
        roller.set(speed);


    }


    public boolean acquired(){


        return sensor.get();
    }
   
    public void periodic(){
        SmartDashboard.putBoolean("Is Object in Roller", acquired());
    }
}


