package frc.robot.controls;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmInterface {
    double speed_limit = Math.PI;

    TalonFX Proximal_Left = new TalonFX(Constants.ArmInterface.proximal_left_ID);
    TalonFX Proximal_Right = new TalonFX(Constants.ArmInterface.proximal_right_ID);

    TalonFX Distal_Left = new TalonFX(Constants.ArmInterface.distal_left_ID);
    TalonFX Distal_Right = new TalonFX(Constants.ArmInterface.distal_right_ID);
    
    DutyCycleEncoder Encoder_Proximal_Left = new DutyCycleEncoder(Constants.ArmInterface.encoder_proximal_left_ID);
    DutyCycleEncoder Encoder_Proximal_Right = new DutyCycleEncoder(Constants.ArmInterface.encoder_proximal_right_ID);

    DutyCycleEncoder Encoder_Distal_Left = new DutyCycleEncoder(Constants.ArmInterface.encoder_distal_left_ID);
    DutyCycleEncoder Encoder_Distal_Right = new DutyCycleEncoder(Constants.ArmInterface.encoder_distal_right_ID);

    public ArmInterface(){
        Proximal_Left.setInverted(Constants.ArmInterface.proximal_left_inversion);
        Proximal_Right.setInverted(Constants.ArmInterface.proximal_right_inversion);
        Distal_Left.setInverted(Constants.ArmInterface.distal_left_inversion);
        Distal_Right.setInverted(Constants.ArmInterface.distal_right_inversion);
    }

    public void resetMotorEncoders(){
        Proximal_Left.setSelectedSensorPosition(0);
        Proximal_Right.setSelectedSensorPosition(0);
        Distal_Left.setSelectedSensorPosition(0);
        Distal_Right.setSelectedSensorPosition(0);
    }
    public double clampAngle(double angle){
        if(angle < 0) angle += 2*Math.PI;
        if(angle > 2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }
    public double getEncoderProximalLeft(){
        double angle = -(Encoder_Proximal_Left.getAbsolutePosition() - Constants.ArmInterface.encoder_proximal_left_offset) * (2 * Math.PI) + Math.PI/2;
        return clampAngle(angle);
    }
    public double getEncoderProximalRight(){
        double angle = (Encoder_Proximal_Right.getAbsolutePosition() - Constants.ArmInterface.encoder_proximal_right_offset) * (2 * Math.PI) + Math.PI/2;
        return clampAngle(angle);
    }
    public double getEncoderDistalLeft(){
        double angle = -(Encoder_Distal_Left.getAbsolutePosition() - Constants.ArmInterface.encoder_distal_left_offset) * (2 * Math.PI) - Math.PI + getEncoderProximalLeft();
        return clampAngle(angle);
    }
    public double getEncoderDistalRight(){
        double angle = (Encoder_Distal_Right.getAbsolutePosition() - Constants.ArmInterface.encoder_distal_right_offset) * (2 * Math.PI) - Math.PI + getEncoderProximalRight();
        return clampAngle(angle);
    }

    public double getMotorEncoderProximalLeft(){
        double angle = Proximal_Left.getSelectedSensorPosition() * Constants.ArmInterface.gear_reduction_proximal + Math.PI/2;
        return clampAngle(angle);
    }
    public double getMotorEncoderProximalRight(){
        double angle = Proximal_Right.getSelectedSensorPosition() * Constants.ArmInterface.gear_reduction_proximal + Math.PI/2;
        return clampAngle(angle);
    }
    public double getMotorEncoderDistalLeft(){
        double angle = Distal_Left.getSelectedSensorPosition() * Constants.ArmInterface.gear_reduction_distal - Math.PI/2;
        return clampAngle(angle);
    }
    public double getMotorEncoderDistalRight(){
        double angle = Distal_Right.getSelectedSensorPosition() * Constants.ArmInterface.gear_reduction_distal - Math.PI/2;
        return clampAngle(angle);
    }

    
}
