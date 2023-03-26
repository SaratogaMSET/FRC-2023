package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ejml.simple.SimpleMatrix;
import frc.robot.Constants;
import frc.robot.controls.ArmInterface;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class IntakeSubsystem extends SubsystemBase {
    WPI_TalonFX Actuator = new WPI_TalonFX(Constants.GroundIntake.actuator_ID);
    WPI_TalonFX Intake = new WPI_TalonFX(Constants.GroundIntake.intake_ID);

    CANCoder Encoder = new CANCoder(Constants.GroundIntake.encoder_ID);

    double get_position(){
        double raw_radians = 2 * Math.PI * (Encoder.getAbsolutePosition() - Constants.GroundIntake.encoder_offset);
        while(raw_radians > Math.PI) raw_radians -= 2 * Math.PI;
        while(raw_radians < -Math.PI) raw_radians += 2 * Math.PI;

        return raw_radians;
    }

    double[] get_xy(){
        return new double[]{
            Constants.GroundIntake.x_offset + Math.sin(get_position()) * Constants.GroundIntake.length,
            Constants.GroundIntake.y_offset + Math.cos(get_position()) * Constants.GroundIntake.length
        };
    }

    double k_G = 0.1;
    double k_P = 0;
    void set_angle(double angle){
        if(angle < Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
        if(angle > Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;
        double error = angle - get_position();

        double gravity_voltage = k_G * Math.cos(get_position());
        Actuator.setVoltage(gravity_voltage + error * k_P);
    }

    void set_intake(double speed){
        Intake.set(speed);
    }

    boolean detect_collision(double[] ee_cartesian, double radius){
        double distance = Math.hypot(ee_cartesian[0] - get_xy()[0], ee_cartesian[1] - get_xy()[1]);

        return Math.abs(distance) < radius;
    }
}
