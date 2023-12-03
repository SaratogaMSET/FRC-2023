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
    WPI_TalonFX actuator = new WPI_TalonFX(Constants.GroundIntake.actuator_ID, "649-Hammerhead-CANivore");
    CANCoder encoder = new CANCoder(Constants.GroundIntake.encoder_ID,  "649-Hammerhead-CANivore");
    //todo: get angle(rad), set angle(rad), account for gravity, set voltage
    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, Constants.Drivetrain.driveContinuousCurrentLimit, GroundIntake.currentLimit, Constants.Drivetrain.drivePeakCurrentDuration);


    double k_p = 0.4;


    double gforce = -0.01; //arbitrary
    PIDController control = new PIDController(k_p, 0, 0);
   


   
    public ActuatorSubsystem(){
        actuator.configSupplyCurrentLimit(limit);
        actuator.setNeutralMode(NeutralMode.Brake);




    }




    public double getRads(){
        double rads = Math.PI/180*(encoder.getAbsolutePosition()- GroundIntake.encoder_offset);
        return rads;
    }
   
    public double getDegrees(){
        double degrees = encoder.getAbsolutePosition()- GroundIntake.encoder_offset;
        return degrees;
    }


    public void setDegrees(double angle, double power){
        double angle_adj;
        double power_adj;
        double adjuster;
        double F_g;
        if(power > 100) power = 100;
        if(power < 0) power = 0;


        if(angle > Constants.GroundIntake.lowbound) angle = Constants.GroundIntake.lowbound;
        if(angle < Constants.GroundIntake.highbound) angle = Constants.GroundIntake.highbound;


        F_g = (gforce*Math.sin(getDegrees()/Constants.GroundIntake.lowbound*90));


        angle_adj = angle/Constants.GroundIntake.lowbound;
        control.setSetpoint(angle_adj);
        adjuster = control.calculate(getDegrees()/Constants.GroundIntake.lowbound);
        power_adj = power/100*adjuster;
       
        if(getDegrees()>angle) power_adj+=F_g;


        actuator.set(power_adj);
    }


    public void stationary(){
        double F_g = (gforce*Math.sin(getDegrees()/Constants.GroundIntake.lowbound*90));
        actuator.set(F_g);


    }














}
