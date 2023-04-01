package frc.robot.subsystems.GroundIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ejml.simple.SimpleMatrix;
import frc.robot.Constants;
import frc.robot.controls.ArmInterface;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class IntakeSubsystem extends SubsystemBase {
    WPI_TalonFX Actuator = new WPI_TalonFX(Constants.GroundIntake.actuator_ID, "649-Hammerhead-CANivore");
    WPI_TalonFX Intake = new WPI_TalonFX(Constants.GroundIntake.intake_ID, "649-Hammerhead-CANivore");

    CANCoder Encoder = new CANCoder(Constants.GroundIntake.encoder_ID,  "649-Hammerhead-CANivore");

    public IntakeSubsystem(){
        Actuator.setNeutralMode(NeutralMode.Brake);
    }
   
    double get_position(){
        double raw_radians =  Math.toRadians(Encoder.getAbsolutePosition() - Constants.GroundIntake.encoder_offset);
        //while(raw_radians > Math.PI) raw_radians -= 2 * Math.PI;
        //while(raw_radians < -Math.PI) raw_radians += 2 * Math.PI;
        // raw_radians = (18.0/60.0) * (16.0/40.0) * (8.0/34.0) * raw_radians;
        return raw_radians;
    }
    public double get_position_degrees(){
        double raw_degrees =  (Encoder.getAbsolutePosition() - Constants.GroundIntake.encoder_offset);
        //while(raw_radians > Math.PI) raw_radians -= 2 * Math.PI;
        //while(raw_radians < -Math.PI) raw_radians += 2 * Math.PI;
        // raw_degrees = (18.0/60.0) * (16.0/40.0) * (8.0/34.0) * raw_degrees;
        return raw_degrees;
    }

    double[] get_xy(){
        return new double[]{
            Constants.GroundIntake.x_offset + Math.sin(get_position()) * Constants.GroundIntake.length,
            Constants.GroundIntake.y_offset + Math.cos(get_position()) * Constants.GroundIntake.length
        };
    }

    double k_G = 0.1;
    double k_P = 0.04;

    /** 0 is considered to be at the top
     * @param desired angle in degrees CW +
     */
    public void set_angle(double angle){
        if(angle > Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
        if(angle < Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;

        double error = angle - get_position_degrees();

        double gravity= k_G * Math.cos(get_position_degrees());

        // if(Math.abs(k_P * error + gravity) > 0.5){
        //     double set = Math.signum(k_P * error + gravity) * 0.5;
        //     System.out.println("Feedback Exceeds Intake");
        //     Actuator.setVoltage(set);
        // }
        // else{
            Actuator.setVoltage(k_P * error + gravity);
        // }
    }

    public void setVoltageActuator(double voltage){
        Actuator.setVoltage(voltage);
    }
    public void set_intake(double speed){
        Intake.set(speed);
    }

    boolean detect_collision(double[] ee_cartesian, double radius){
        double distance = Math.hypot(ee_cartesian[0] - get_xy()[0], ee_cartesian[1] - get_xy()[1]);

        return Math.abs(distance) < radius;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Intake Encoder",Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Ground Intake Angle", get_position_degrees());
        SmartDashboard.putNumber("Actuator Voltage", Actuator.getMotorOutputVoltage());
        SmartDashboard.putNumber("Intake Error",  k_G * Math.cos(get_position_degrees() + (90 - get_position_degrees()) * k_P));
    }
}