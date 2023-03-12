package frc.robot.controls;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.ArmParameters;

import org.ejml.simple.SimpleMatrix;

public class ArmInterface {
    double motor_encoder_agreement_tolerance = 0.03;

    WPI_TalonFX Proximal_Left = new WPI_TalonFX(Constants.ArmParameters.proximal_left_ID);
    WPI_TalonFX Proximal_Right = new WPI_TalonFX(Constants.ArmParameters.proximal_right_ID);

    WPI_TalonFX Distal_Left = new WPI_TalonFX(Constants.ArmParameters.distal_left_ID);
    WPI_TalonFX Distal_Right = new WPI_TalonFX(Constants.ArmParameters.distal_right_ID);
    
    CANCoder Encoder_Proximal_Left = new CANCoder(Constants.ArmParameters.encoder_proximal_left_ID);
    CANCoder Encoder_Proximal_Right = new CANCoder(Constants.ArmParameters.encoder_proximal_right_ID);

    CANCoder Encoder_Distal_Left = new CANCoder(Constants.ArmParameters.encoder_distal_left_ID);
    CANCoder Encoder_Distal_Right = new CANCoder(Constants.ArmParameters.encoder_distal_right_ID);

    VelocityFromEncoder EncoderV_Proximal_Left = new VelocityFromEncoder(getEncoderProximalLeft(), 7);
    VelocityFromEncoder EncoderV_Proximal_Right = new VelocityFromEncoder(getEncoderProximalRight(), 7);
    VelocityFromEncoder EncoderV_Distal_Left = new VelocityFromEncoder(getEncoderDistalLeft(), 7);
    VelocityFromEncoder EncoderV_Distal_Right = new VelocityFromEncoder(getEncoderDistalRight(), 7);

    private final double[] Bounds_Proxima = new double[]{Constants.ArmParameters.proximal_lowbound, Constants.ArmParameters.proximal_highbound};
    private final double[] Bounds_Distal = new double[]{Constants.ArmParameters.distal_lowbound, Constants.ArmParameters.distal_highbound};

    Joint proximal = new Joint(Constants.ArmParameters.proximal_length, Constants.ArmParameters.proximal_mass, Constants.ArmParameters.proximal_inertia, Constants.ArmParameters.proximal_com);
    Joint distal = new Joint(Constants.ArmParameters.distal_length, Constants.ArmParameters.distal_mass, Constants.ArmParameters.distal_inertia, Constants.ArmParameters.distal_com);

    public ArmMassControl ArmControl = new ArmMassControl(proximal, distal);
    public ArmKinematics Arm = new ArmKinematics(proximal, distal);

    public ArmInterface(){
        boolean coasted = false;
        if(coasted){
            Proximal_Left.setNeutralMode(NeutralMode.Coast);
            Proximal_Right.setNeutralMode(NeutralMode.Coast);
            Distal_Left.setNeutralMode(NeutralMode.Coast);
            Distal_Right.setNeutralMode(NeutralMode.Coast);
        }else{
            Proximal_Left.setNeutralMode(NeutralMode.Brake);
            Proximal_Right.setNeutralMode(NeutralMode.Brake);
            Distal_Left.setNeutralMode(NeutralMode.Brake);
            Distal_Right.setNeutralMode(NeutralMode.Brake);
        }
        

        Proximal_Left.setInverted(Constants.ArmParameters.proximal_left_inversion);
        Proximal_Right.setInverted(Constants.ArmParameters.proximal_right_inversion);
        Distal_Left.setInverted(Constants.ArmParameters.distal_left_inversion);
        Distal_Right.setInverted(Constants.ArmParameters.distal_right_inversion);
    }
    
    //VERIFY MANUALLY BEFORE RESETTING
    public void resetMotorEncoders(){
        double ProximalMotorEncoderRatio = Constants.ArmParameters.gear_reduction_proximal * Constants.ArmParameters.motor_encoder_ticks_per_revolution / (2 * Math.PI);
        double DistalMotorEncoderRatio = Constants.ArmParameters.gear_reduction_distal * Constants.ArmParameters.motor_encoder_ticks_per_revolution / (2 * Math.PI);
        Proximal_Left.setSelectedSensorPosition(getEncoderProximalLeft() * ProximalMotorEncoderRatio);
        Proximal_Right.setSelectedSensorPosition(-getEncoderProximalRight() * ProximalMotorEncoderRatio);
        Distal_Left.setSelectedSensorPosition(getEncoderDistalLeft() * DistalMotorEncoderRatio);
        Distal_Right.setSelectedSensorPosition(-getEncoderDistalRight() * DistalMotorEncoderRatio);

        while(Math.abs(Math.abs(getEncoderProximalLeft() - getMotorEncoderProximalLeft()) - Math.PI) < motor_encoder_agreement_tolerance){
            Proximal_Left.setSelectedSensorPosition(Proximal_Left.getSelectedSensorPosition() + Math.signum(getEncoderProximalLeft() - getMotorEncoderProximalLeft()) * Math.PI * ProximalMotorEncoderRatio);
        }
        while(Math.abs(Math.abs(getEncoderProximalRight() - getMotorEncoderProximalRight()) - Math.PI) < motor_encoder_agreement_tolerance){
            Proximal_Right.setSelectedSensorPosition(Proximal_Right.getSelectedSensorPosition() + Math.signum(getEncoderProximalRight() - getMotorEncoderProximalRight()) * Math.PI * ProximalMotorEncoderRatio);
        }
        while(Math.abs(Math.abs(getEncoderDistalLeft() - getMotorEncoderDistalLeft()) - Math.PI) < motor_encoder_agreement_tolerance){
            Distal_Left.setSelectedSensorPosition(Distal_Left.getSelectedSensorPosition() + Math.signum(getEncoderDistalLeft() - getMotorEncoderDistalLeft()) * Math.PI * DistalMotorEncoderRatio);
        }
        while(Math.abs(Math.abs(getEncoderDistalRight() - getMotorEncoderDistalRight()) - Math.PI) < motor_encoder_agreement_tolerance){
            Distal_Right.setSelectedSensorPosition(Distal_Right.getSelectedSensorPosition() + Math.signum(getEncoderDistalRight() - getMotorEncoderDistalRight()) * Math.PI * DistalMotorEncoderRatio);
        }
    }

    public double clampAngle(double angle){
        while(angle < 0) angle += 2*Math.PI;
        while(angle > 2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }
    public double clampDistal(double angle){
        while(angle < -3*Math.PI/2) angle += 2*Math.PI;
        while(angle > Math.PI/2) angle -= 2*Math.PI;
        return angle;
    }
    public double getEncoderProximalLeft(){
        double angle = (Encoder_Proximal_Left.getAbsolutePosition()/360 - Constants.ArmParameters.encoder_proximal_left_offset) * (2 * Math.PI) + Math.PI/2;
        return clampAngle(angle);
    }
    public double getEncoderProximalRight(){
        double angle = -(Encoder_Proximal_Right.getAbsolutePosition()/360 - Constants.ArmParameters.encoder_proximal_right_offset) * (2 * Math.PI) + Math.PI/2;
        return clampAngle(angle);
    }
    public double getEncoderDistalLeft(){
        double angle = (Encoder_Distal_Left.getAbsolutePosition()/360 - Constants.ArmParameters.encoder_distal_left_offset) * (2 * Math.PI) - Math.PI + getEncoderProximalLeft();
        return clampDistal(angle);
    }
    public double getEncoderDistalRight(){
        double angle = -(Encoder_Distal_Right.getAbsolutePosition()/360 - Constants.ArmParameters.encoder_distal_right_offset) * (2 * Math.PI) - Math.PI + getEncoderProximalRight();
        return clampDistal(angle);
    }

    public double getMotorEncoderProximalLeft(){
        double angle = 2 * Math.PI * Proximal_Left.getSelectedSensorPosition() / Constants.ArmParameters.gear_reduction_proximal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
        return clampAngle(angle);
    }
    public double getMotorEncoderProximalRight(){
        double angle = 2 * Math.PI * Proximal_Right.getSelectedSensorPosition() / Constants.ArmParameters.gear_reduction_proximal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
        return clampAngle(angle);
    }
    public double getMotorEncoderDistalLeft(){
        double angle = 2 * Math.PI * Distal_Left.getSelectedSensorPosition() / Constants.ArmParameters.gear_reduction_distal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
        return clampDistal(angle);
    }
    public double getMotorEncoderDistalRight(){
        double angle = 2 * Math.PI * Distal_Right.getSelectedSensorPosition() / Constants.ArmParameters.gear_reduction_distal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
        return clampDistal(angle);
    }

    public double getEncoderVProximalLeft(){
        return EncoderV_Proximal_Left.getRawVelocity();
    }
    public double getEncoderVProximalRight(){
        return EncoderV_Proximal_Right.getRawVelocity();
    }
    public double getEncoderVDistalLeft(){
        return EncoderV_Distal_Left.getRawVelocity();
    }
    public double getEncoderVDistalRight(){
        return EncoderV_Distal_Right.getRawVelocity();
    }

    public double getMotorEncoderVProximalLeft(){
        return 2 * Math.PI * 10 * Proximal_Left.getSelectedSensorVelocity() / Constants.ArmParameters.gear_reduction_proximal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
    }
    public double getMotorEncoderVProximalRight(){
        return 2 * Math.PI * 10 * Proximal_Right.getSelectedSensorVelocity() / Constants.ArmParameters.gear_reduction_proximal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
    }
    public double getMotorEncoderVDistalLeft(){
        return 2 * Math.PI * 10 * Distal_Left.getSelectedSensorVelocity() / Constants.ArmParameters.gear_reduction_distal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
    }
    public double getMotorEncoderVDistalRight(){
        return 2 * Math.PI * 10 * Distal_Right.getSelectedSensorVelocity() / Constants.ArmParameters.gear_reduction_distal / Constants.ArmParameters.motor_encoder_ticks_per_revolution;
    }

    public double getPositionProximalLeft(){
        if(Math.abs(getEncoderProximalLeft() - getMotorEncoderProximalLeft()) < motor_encoder_agreement_tolerance){
            return getEncoderProximalLeft();
        }
        return getMotorEncoderProximalLeft();
    }
    public double getPositionProximalRight(){
        if(Math.abs(getEncoderProximalRight() - getMotorEncoderProximalRight()) < motor_encoder_agreement_tolerance){
            return getEncoderProximalRight();
        }
        return getMotorEncoderProximalRight();
    }
    public double getPositionDistalLeft(){
        if(Math.abs(getEncoderDistalLeft() - getMotorEncoderDistalLeft()) < motor_encoder_agreement_tolerance){
            return getEncoderDistalLeft();
        }
        return getMotorEncoderDistalLeft();
    }
    public double getPositionDistalRight(){
        if(Math.abs(getEncoderDistalRight() - getMotorEncoderDistalRight()) < motor_encoder_agreement_tolerance){
            return getEncoderDistalRight();
        }
        return getMotorEncoderDistalRight();
    }


    public double getVelocityProximalLeft(){
        if(Math.abs(getEncoderProximalLeft() - getMotorEncoderProximalLeft()) < motor_encoder_agreement_tolerance){
            return getEncoderVProximalLeft();
        }
        return getMotorEncoderVProximalLeft();
    }
    public double getVelocityProximalRight(){
        if(Math.abs(getEncoderProximalRight() - getMotorEncoderProximalRight()) < motor_encoder_agreement_tolerance){
            return getEncoderVProximalRight();
        }
        return getMotorEncoderVProximalRight();
    }
    public double getVelocityDistalLeft(){
        if(Math.abs(getEncoderDistalLeft() - getMotorEncoderDistalLeft()) < motor_encoder_agreement_tolerance){
            return getEncoderVDistalLeft();
        }
        return getMotorEncoderVDistalLeft();
    }
    public double getVelocityDistalRight(){
        if(Math.abs(getEncoderDistalRight() - getMotorEncoderDistalRight()) < motor_encoder_agreement_tolerance){
            return getEncoderVDistalRight();
        }
        return getMotorEncoderVDistalRight();
    }
    
    public double getPositionProximal(){
        //return (getPositionProximalLeft() + getPositionProximalRight())/2;
        return (getEncoderProximalLeft() + getEncoderProximalRight())/2;
    }
    public double getPositionDistal(){
        return (getEncoderDistalLeft() + getEncoderDistalRight())/2;
    }
    public double getVelocityProximal(){
        return (getMotorEncoderVProximalLeft() + getMotorEncoderVProximalRight())/2;
    }
    public double getVelocityDistal(){
        return (getMotorEncoderVDistalLeft() + getMotorEncoderVDistalRight())/2;
    }

    public SimpleMatrix state(){
        return new SimpleMatrix(new double[][]{{getPositionProximal()}, {getPositionDistal()}, {getVelocityProximal()}, {getVelocityDistal()}});
    }
    public SimpleMatrix state_nv(){
        return new SimpleMatrix(new double[][]{{getPositionProximal()}, {getPositionDistal()}, {0}, {0}});
    }
    public void update_state(){
        EncoderV_Proximal_Left.update(getEncoderProximalLeft());
        EncoderV_Proximal_Right.update(getEncoderProximalRight());
        EncoderV_Distal_Left.update(getEncoderDistalLeft());
        EncoderV_Distal_Right.update(getEncoderDistalRight());

        Arm.setAngles(getPositionProximal(), getPositionDistal());
        Arm.setOmegas(getVelocityProximal(), getVelocityDistal());
    }

    public void showState(){
        double radians_to_degrees = 180 / Math.PI;
        String raw_values = "Arm Raw Encoders/";
        boolean show_raw_values = false;
        if(show_raw_values){
            SmartDashboard.putNumber(raw_values + "Proximal Left", Encoder_Proximal_Left.getAbsolutePosition()/360);
            SmartDashboard.putNumber(raw_values + "Proximal Right", Encoder_Proximal_Right.getAbsolutePosition()/360);
            SmartDashboard.putNumber(raw_values + "Distal Left", Encoder_Distal_Left.getAbsolutePosition()/360);
            SmartDashboard.putNumber(raw_values + "Distal Right", Encoder_Distal_Right.getAbsolutePosition()/360);
        }

        String encoder_values = "Arm External Encoders/";
        boolean show_encoder_values = true;
        if(show_encoder_values){
            SmartDashboard.putNumber(encoder_values + "Proximal Left", getEncoderProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_values + "Proximal Right", getEncoderProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_values + "Distal Left", getEncoderDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_values + "Distal Right", getEncoderDistalRight() * radians_to_degrees);
        }

        String motor_encoder_values = "Arm Motor Encoders/";
        boolean show_motor_encoder_values = true;
        if(show_motor_encoder_values){
            SmartDashboard.putNumber(motor_encoder_values + "Proximal Left", getMotorEncoderProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_values + "Proximal Right", getMotorEncoderProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_values + "Distal Left", getMotorEncoderDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_values + "Distal Right", getMotorEncoderDistalRight() * radians_to_degrees);
        }

        String encoder_velocity_values = "Arm External Encoder Velocities/";
        boolean show_encoder_velocity_values = false;
        if(show_encoder_velocity_values){
            SmartDashboard.putNumber(encoder_velocity_values + "Proximal Left", getEncoderVProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_velocity_values + "Proximal Right", getEncoderVProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_velocity_values + "Distal Left", getEncoderVDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(encoder_velocity_values + "Distal Right", getEncoderVDistalRight() * radians_to_degrees);
        }

        String motor_encoder_velocity_values = "Arm Motor Encoders Velocities/";
        boolean show_motor_encoder_velocity_values = false;
        if(show_motor_encoder_velocity_values){
            SmartDashboard.putNumber(motor_encoder_velocity_values + "Proximal Left", getMotorEncoderVProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_velocity_values + "Proximal Right", getMotorEncoderVProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_velocity_values + "Distal Left", getMotorEncoderVDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(motor_encoder_velocity_values + "Distal Right", getMotorEncoderVDistalRight() * radians_to_degrees);
        }

        // String motor_encoder_agreement = "Arm Motor + ExtEnc Agreement/";
        // boolean show_motor_encoder_agreement = true;
        // if(show_motor_encoder_agreement){
        //     SmartDashboard.putBoolean(motor_encoder_agreement + "Proximal Left", Math.abs(getEncoderProximalLeft() - getMotorEncoderProximalLeft()) < motor_encoder_agreement_tolerance);
        //     SmartDashboard.putBoolean(motor_encoder_agreement + "Proximal Right", Math.abs(getEncoderProximalRight() - getMotorEncoderProximalRight()) < motor_encoder_agreement_tolerance);
        //     SmartDashboard.putBoolean(motor_encoder_agreement + "Distal Left", Math.abs(getEncoderDistalLeft() - getMotorEncoderDistalLeft()) < motor_encoder_agreement_tolerance);
        //     SmartDashboard.putBoolean(motor_encoder_agreement + "Distal Right", Math.abs(getEncoderDistalRight() - getMotorEncoderDistalRight()) < motor_encoder_agreement_tolerance);
        // }

        String arm_perception = "Arm Microstate/";
        boolean show_arm_perception = false;
        if(show_arm_perception){
            String posdiv = "Pos/";
            SmartDashboard.putNumber(arm_perception + posdiv + "Proximal Left", getPositionProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + posdiv + "Proximal Right", getPositionProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + posdiv + "Distal Left", getPositionDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + posdiv + "Distal Right", getPositionDistalRight() * radians_to_degrees);

            String veldiv = "Vel/";
            SmartDashboard.putNumber(arm_perception + veldiv + "Proximal Left", getVelocityProximalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + veldiv + "Proximal Right", getVelocityProximalRight() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + veldiv + "Distal Left", getVelocityDistalLeft() * radians_to_degrees);
            SmartDashboard.putNumber(arm_perception + veldiv + "Distal Right", getVelocityDistalRight() * radians_to_degrees);
        }

        // String macro_view = "Arm State/";
        // boolean show_macroview = true;
        // if(show_macroview){
        //     String posdiv = "Pos/";
        //     SmartDashboard.putNumber(macro_view + posdiv + "Proximal", getPositionProximal() * radians_to_degrees);
        //     SmartDashboard.putNumber(macro_view + posdiv + "Distal", getPositionDistal() * radians_to_degrees);

        //     String veldiv = "Vel/";
        //     SmartDashboard.putNumber(macro_view + veldiv + "Proximal", getVelocityProximal() * radians_to_degrees);
        //     SmartDashboard.putNumber(macro_view + veldiv + "Distal", getVelocityDistal() * radians_to_degrees);
        // }

        // String erroneous_forces = "Arm Erroneous Forces/";
        // boolean show_erroneous_forces = true;
        // if(show_erroneous_forces){
        //     SimpleMatrix feedforward = ArmControl.counteractErroneousForces(state());
        //     SmartDashboard.putNumber(erroneous_forces + "Proximal FF", feedforward.get(0));
        //     SmartDashboard.putNumber(erroneous_forces + "Distal FF", feedforward.get(1));

        //     double feedforward_proximal_voltage = VoltageControl.GearedF500Voltage(0.5 * feedforward.get(0), getVelocityProximal(), 0, Constants.ArmParameters.gear_reduction_proximal);
        //     double feedforward_distal_voltage = VoltageControl.GearedF500Voltage(0.5 * feedforward.get(1), getVelocityDistal(), 0, Constants.ArmParameters.gear_reduction_distal);

        //     SmartDashboard.putNumber(erroneous_forces + "Proximal FF Volts", feedforward_proximal_voltage);
        //     SmartDashboard.putNumber(erroneous_forces + "Distal FF Volts", feedforward_distal_voltage);
        // }

        // String kinematics = "Arm Kinematics/";
        // boolean show_kinematics = true;
        // if(show_kinematics){
        //     String forward = "FWD/";
        //     double[] X = Arm.forwardKinematics(getPositionProximal(), getPositionDistal());
        //     SmartDashboard.putNumber(kinematics + forward + "X", X[0]);
        //     SmartDashboard.putNumber(kinematics + forward + "Y", X[1]);

        //     String inverse = "INV/";
        //     double[] Q = Arm.inverseKinematics(X[0], X[1]);
        //     SmartDashboard.putBoolean(kinematics + inverse + "Q1", Math.abs(Q[0] - getPositionProximal()) < 0.001);
        //     SmartDashboard.putBoolean(kinematics + inverse + "Q2", Math.abs(Q[1] - getPositionDistal()) < 0.001);
        // }

        // String bounds = "Arm Bounds/";
        // boolean show_bounds = true;
        // if(show_bounds){
        //     SmartDashboard.putBoolean(bounds + "Proximal Low", getPositionProximal() < Bounds_Proxima[0]);
        //     SmartDashboard.putBoolean(bounds + "Proximal High", getPositionProximal() > Bounds_Proxima[1]);
        //     SmartDashboard.putBoolean(bounds + "Distal Low", getPositionDistal() < Bounds_Distal[0]);
        //     SmartDashboard.putBoolean(bounds + "Distal High", getPositionDistal() > Bounds_Distal[1]);
        // }
    }
    public void voltageMotors_SimpleFF(double controlVoltageProxima, double controlVoltageDistal){
        if(getPositionProximal() < Bounds_Proxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getPositionProximal() > Bounds_Proxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getPositionDistal() < Bounds_Distal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getPositionDistal() > Bounds_Distal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;

        double proximal_kVG = 1.20;
        double distal_kVG = 0.66;

        double voltage_ff_prox = Math.cos(getPositionProximal()) * proximal_kVG;
        double voltage_ff_dist = Math.cos(getPositionDistal()) * distal_kVG;

        if(controlVoltageProxima == 0 && controlVoltageDistal == 0){

        }

        Proximal_Left.setVoltage(controlVoltageProxima + voltage_ff_prox);
        Proximal_Right.setVoltage(controlVoltageProxima + voltage_ff_prox);

        Distal_Left.setVoltage(controlVoltageDistal + voltage_ff_dist);
        Distal_Right.setVoltage(controlVoltageDistal + voltage_ff_dist);
    }
    public void voltageMotors(double controlVoltageProxima, double controlVoltageDistal){
        if(getPositionProximal() < Bounds_Proxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getPositionProximal() > Bounds_Proxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getPositionDistal() < Bounds_Distal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getPositionDistal() > Bounds_Distal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;

        SimpleMatrix counteractingTorquesErroneous = ArmControl.counteractErroneousForces(state_nv());

        double tq_ff_prox = counteractingTorquesErroneous.get(0);
        double tq_ff_dist = counteractingTorquesErroneous.get(1);
        double voltage_ff_prox = VoltageControl.GearedF500Voltage(tq_ff_prox * 3.2, 0, 0, Constants.ArmParameters.gear_reduction_proximal);
        double voltage_ff_dist = VoltageControl.GearedF500Voltage(tq_ff_dist * 3.2, 0, 0, Constants.ArmParameters.gear_reduction_distal);

        double max_ff = 2;

        if(Math.abs(voltage_ff_prox) > max_ff) {
            voltage_ff_prox = 0;
            System.out.println("Feedforward Error Proximal");
        }
        if(Math.abs(voltage_ff_dist) > max_ff) {
            voltage_ff_dist = 0;
            System.out.println("Feedforward Error Distal");
        }

        Proximal_Left.setVoltage(controlVoltageProxima + voltage_ff_prox);
        Proximal_Right.setVoltage(controlVoltageProxima + voltage_ff_prox);

        Distal_Left.setVoltage(controlVoltageDistal + voltage_ff_dist);
        Distal_Right.setVoltage(controlVoltageDistal + voltage_ff_dist);
    }
    /*
    public void rawVoltageMotors(double controlVoltageProxima, double controlVoltageDistal){
        if(getProximalRadians() < boundsProxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getProximalRadians() > boundsProxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getDistalRadians() < boundsDistal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getDistalRadians() > boundsDistal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;
        motorProximal.setVoltage(controlVoltageProxima);
        motorDistal.setVoltage(controlVoltageDistal);
    }

     */
}