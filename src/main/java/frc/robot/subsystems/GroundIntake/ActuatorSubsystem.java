package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntake;

public class ActuatorSubsystem extends SubsystemBase {
    WPI_TalonFX Actuator = new WPI_TalonFX(Constants.GroundIntake.actuator_ID, "649-Hammerhead-CANivore");

    CANCoder Encoder = new CANCoder(Constants.GroundIntake.encoder_ID,  "649-Hammerhead-CANivore");
    //TODO: Make these constants not completely bogus
    //They were here because the intake was added under a time constraint, they are not "valid" control theory wise
    double previousError = 0;
    public double k_GAuton = 0.0648; //0.0648
    public double k_G = 0.0648;
    double k_PAuton = 0.85;
    double k_P = 0.6; //0.65
    double k_D = 0.000;
    double k_I = 0.000;
    double k_PUp = 0.6; //0.7
    double errorDT;
    double prevError;

    //Ki is almost always not a good idea on a frc robot
    PIDController controller = new PIDController(k_P,0,k_D);
    
    //How we current limit the motor actuating the ground intake 
    SupplyCurrentLimitConfiguration ActuatorLimit = new SupplyCurrentLimitConfiguration(
            true, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            GroundIntake.currentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);
    public ActuatorSubsystem(){
        Actuator.setNeutralMode(NeutralMode.Brake);
        Actuator.configSupplyCurrentLimit(ActuatorLimit);
        Actuator.configNeutralDeadband(0.0);
    }
   

    //Very defined range of motion 
    public double get_position(){
        double raw_radians =  Math.toRadians(Encoder.getAbsolutePosition() - Constants.GroundIntake.encoder_offset);
        return raw_radians;
    }
    public double get_position_degrees(){
        double raw_degrees =  (Encoder.getAbsolutePosition() - Constants.GroundIntake.encoder_offset);
        //while(raw_radians > Math.PI) raw_radians -= 2 * Math.PI;
        //while(raw_radians < -Math.PI) raw_radians += 2 * Math.PI;
        // raw_degrees = (18.0/60.0) * (16.0/40.0) * (8.0/34.0) * raw_degrees;
        return raw_degrees;
    }
    public double getCurrent(){
        return Actuator.getStatorCurrent();
    }

    // /**
    //  * Does not do anything useful for right now
    //  * 
    //  */
    // double[] get_xy(){
    //     return new double[]{
    //         Constants.GroundIntake.x_offset + Math.sin(get_position()) * Constants.GroundIntake.length,
    //         Constants.GroundIntake.y_offset + Math.cos(get_position()) * Constants.GroundIntake.length
    //     };
    // }

    public double getVoltage(){
        return Actuator.getMotorOutputVoltage();
    }

    /** 0 is considered to be the ground intake at vertical
     * @param desired angle in degrees CW +
     */
    public void set_angle(double angle, double powerPercent){
        if (powerPercent > 100) powerPercent = 100;
        if (powerPercent < 0) powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if(angle > Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
        if(angle < Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;
        double error = (angle - get_position_degrees())/(Constants.GroundIntake.lowbound - Constants.GroundIntake.highbound);
        double gravity = k_GAuton * Math.sin(get_position() - Math.toRadians(25));
        
        if(angle < get_position_degrees()){
            Actuator.setVoltage((k_PUp * error * power) - gravity);
        }
        else{
            Actuator.setVoltage(((k_PAuton * error)* power));
        }
        // SmartDashboard.putNumber("Intake Error",  (k_P * error * power));
} 

    // public void set_angle(double angle, double powerPercent, double velocityError){
    //     if (powerPercent > 100) powerPercent = 100;
    //     if (powerPercent < 0) powerPercent = 0;

    //     double power = 12 * powerPercent / 100;
    //     if(angle > Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
    //     if(angle < Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;
    //     double error = (angle - get_position_degrees())/(Constants.GroundIntake.lowbound - Constants.GroundIntake.highbound);
    //     double gravity = k_G * Math.sin(get_position() - Math.toRadians(25));
        
    //     if(angle < get_position_degrees()){
    //         Actuator.setVoltage((k_PUp * error * power) - gravity);
    //     }
    //     else{
    //         Actuator.setVoltage(((k_P * error + k_D * velocityError )* power));
    //     }
    // } 

    public void set_angle(double angle, double powerPercent, double integralError, double velocityError){
        if (powerPercent > 100) powerPercent = 100;
        if (powerPercent < 0) powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if(angle > Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
        if(angle < Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;
        double error = (angle - get_position_degrees())/(Constants.GroundIntake.lowbound - Constants.GroundIntake.highbound);
        
        double gravity = k_G * Math.sin(get_position() - Math.toRadians(25));
        
        if(angle < get_position_degrees()){
            Actuator.setVoltage((k_PUp * error * power) - gravity);
        }
        else{
            
            double outputVoltage = (k_P * error + k_I * integralError + k_D * velocityError- gravity) * power;
            // SmartDashboard.putNumber("More Pain", outputVoltage );

            //we slam the ground intake down if we dont limit the voltage of the motor
            double calc = Math.max(0.5, Math.min(4, outputVoltage));
            Actuator.setVoltage(calc);
        }
    } 


    // public void setVoltageActuator(double voltage){
    //     Actuator.setVoltage(voltage);
    // }

    /**
     * Does not do anything useful for right now
     */
    // public boolean detect_collision(double[] ee_cartesian, double radius){
    //     // double distance = Math.hypot(ee_cartesian[0] - get_xy()[0], ee_cartesian[1] - get_xy()[1]);

    //     return Math.abs(distance) < radius;
    // }

    /** Keeps the ground intake up.
     * This should be done in a command that is almost always running, like a default command,
     * so that the ground intake does not start to droop on its own.
     */
    public void gravityCompensation(){
        Actuator.set(-k_G * Math.sin(get_position() - Math.toRadians(25)));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Intake Angle", get_position_degrees());
        // SmartDashboard.putNumber("Ground Intake Angle", get_position_degrees());
        // SmartDashboard.putNumber("Actuator Voltage", Actuator.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Gravity", k_G * Math.sin(get_position() - Math.toRadians(25)));
        // SmartDashboard.putNumberArray("Intake Coors", get_xy());
        
    }
}