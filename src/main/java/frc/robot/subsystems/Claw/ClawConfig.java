package frc.robot.subsystems.Claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class ClawConfig {
    public static DigitalInput limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH);
    public static CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    public static RelativeEncoder encoder = motor.getEncoder();
    public ClawConfig(){
        motor.setIdleMode(IdleMode.kBrake);
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(360.0 / 320 * 3);
        encoder.setPosition(0);
    }
}
