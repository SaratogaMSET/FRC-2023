package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;
import frc.robot.Constants;

public class ActuatorSubsystem extends SubsystemBase {
    //PID Controller Values

    //You want this value to be just enough such that the angle is as close to the setpoint as possible but not overshooting.
    //Increasing will increase the power/ make it overshoot more, decreasing it lowers the power.
    double kp = 0.37;

    double ki = 0; //Don't touch this.
    double kd = 0.0; //Changing this made the ground intake oscillate too much
    double kg = 0.025; //Changes how much power you need to overcome gravity
    PIDController controller = new PIDController(kp, ki, kd); 

    //Encoders essentially save the position of the motor, in this case saving the angle of the actuator
    CANCoder encoder = new CANCoder(GroundIntake.encoder_ID, "649-Hammerhead-CANivore");
    WPI_TalonFX motor = new WPI_TalonFX(GroundIntake.actuator_ID, "649-Hammerhead-CANivore");
    //Setting current limits as a failsafe
    SupplyCurrentLimitConfiguration ActuatorLimit = new SupplyCurrentLimitConfiguration(
            true, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            GroundIntake.currentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);
    public ActuatorSubsystem() {
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSupplyCurrentLimit(ActuatorLimit);
        motor.configNeutralDeadband(0.0);
    }
    //Encoder Offset is used to make the odometry match up with the position of the actuator in real life
    public double getAngle() {
        return encoder.getAbsolutePosition() - GroundIntake.encoder_offset;
    }
    public void setAngle(double angle, double powerPercent) {
        powerPercent = MathUtil.clamp(powerPercent, 0, 100); //clamp(value, low, high) returns low if the value is lower than low, high if the value is higher than high, else just return the value.
        angle = MathUtil.clamp(angle, GroundIntake.highbound, GroundIntake.lowbound); //Names are a little misleading, lowbound is a large number as the angle when the actuator is close to the ground is high and vice versa.
        double scaledAngle = angle / GroundIntake.lowbound; //Scaling the angle so it is essentially a percentage of how far the angle is from lowbound, needed for PID controller calculations.
        controller.setSetpoint(scaledAngle); //Sets the PID Controller setpoint to the scaled angle
        double power = (powerPercent / 100) * controller.calculate(getAngle() / GroundIntake.lowbound); //Scaling the measurement of the current angle so controller.calculate returns a value between -1 and 1.

        //Calculating gravity is a little math-heavy.
        //The entire value must be negated, as bringing the actuator closer to the arm needs negative motor power
        // Because we want the closer the actuator is to the ground, the more power gravity will have, we use sin here.
        // Because the angle of our actuator can go below 90 sin(90 to 180 degrees) is actually decreasing, so we scale the angle to only be between 0 and 90
        double gravity = (-kg * Math.sin(Math.toRadians(getAngle()/GroundIntake.lowbound*90))); 
        if (getAngle() > angle) { //If our current angle is closer to the ground than the target angle, we use extra power due to gravity
            power += gravity;
        }
        motor.set(power);
    }
    public void idle() {
        //Uses the gravity power from setAngle in order to not have the actuator droop down
        motor.set(-kg * Math.sin(Math.toRadians(getAngle()/GroundIntake.lowbound * 90))); 
    }
    @Override
    public void periodic() {
        //Puts values out where you can see them on the driver station, very useful for debugging!
        SmartDashboard.putNumber("Angle of Actuator", getAngle());
        SmartDashboard.putNumber("Scaled Angle of Actuator", getAngle() / GroundIntake.lowbound);
        SmartDashboard.putNumber("Actutor Motor power", motor.get());
    }
}