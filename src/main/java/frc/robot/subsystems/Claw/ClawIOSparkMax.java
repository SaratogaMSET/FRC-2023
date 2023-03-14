// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Claw.ClawSubsystem.Objects;

public class ClawIOSparkMax extends SubsystemBase implements ClawIO {
    private double[] proximityBuffer = new double[3];
    private int bufferIndex = 0;
    private double cubeBlueTheshold = 0.27;
    private Objects objectState = Objects.None;
    private Color currentColor;
    boolean previousLimitSwitch = true;
    public static ColorSensorV3 colorSensor;
    // public static DigitalInput HallEffect = new DigitalInput(IntakeConstants.HALL_EFFECT);
    public static CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    public static RelativeEncoder encoder = motor.getEncoder();
    String print = "Claw/"; 

    public ClawIOSparkMax() {
        motor.setIdleMode(IdleMode.kBrake);
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setMotorVoltage(double voltage) {
       
    }

    public void updateInputs(ClawIOInputsAutoLogged inputs) {
        inputs.rotations = encoder.getPosition();
        inputs.objectDetected = objectInRange();
        inputs.proximity = proximityValue();
        inputs.object = getObject().toString();
    }
    
    public void setBrakeMode(){
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void openIntake() {
        updateIntake();
        double velocitySetpoint;
        if(getReverseLimitSwitch()){
            velocitySetpoint = 0.0;
            resetEncoder();
        }
        else{ 
            velocitySetpoint = -Constants.IntakeConstants.TARGET_VELOCITY;
        }
        SmartDashboard.putNumber("Target velocity", velocitySetpoint);
        motor.set(velocitySetpoint);
    }

    // public void openIntake(double openTo){
    //     updateIntake();
    //     double velocitySetpoint = getForwardLimitSwitch() || encoder.getPosition() <= -openTo ? 0.0
    //             : -Constants.IntakeConstants.TARGET_VELOCITY;
    //     SmartDashboard.putNumber("Target velocity", velocitySetpoint);
    //     motor.set(velocitySetpoint);
    // }

    public void setIdle() {
        motor.set(0.0);
        previousLimitSwitch = true;
    }

    private void updateProximitySensor() {
        proximityBuffer[bufferIndex] = colorSensor.getProximity();
        bufferIndex++;
        bufferIndex = bufferIndex % proximityBuffer.length;
    }

    private double proximityValue() {
        updateProximitySensor();
        double sum = 0;
        for (double x : proximityBuffer)
            sum += x;
        return sum / proximityBuffer.length;
    }

    public boolean objectInRange() {
        return proximityValue() > 65;
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    public void autoCloseIntake(){
        if(objectInRange()) {

            if (!getReverseLimitSwitch() && previousLimitSwitch) { 
                // If the claw is not fully open & if we need to open the claw, open claw
                openIntake();
                // previousLimitSwitch = getReverseLimitSwitch();
                return;
            }
            else {
                previousLimitSwitch = false;
                double encoderPosition = encoder.getPosition();
                if(getObject() == Objects.Cube){
                    if(encoderPosition >= IntakeConstants.CUBE_MEDIUM_BOUND-1){
                        motor.set(0.0);
                    }
                    else{
                        motor.set(IntakeConstants.TARGET_VELOCITY);
                    }
                
                }
                else if(getObject() == Objects.Cone){
                    if(encoderPosition >= IntakeConstants.CONE_MEDIUM_BOUND-1){
                        motor.set(0.0);
                    }
                    else{
                        motor.set(IntakeConstants.TARGET_VELOCITY);
                    }
                }
                else{
                    motor.set(0.0);
                }

                SmartDashboard.putBoolean(print + "Game Piece Object", getObject() == Objects.Cube);
                SmartDashboard.putBoolean(print + "cube bound", encoderPosition >= IntakeConstants.CUBE_MEDIUM_BOUND-1);
                SmartDashboard.putBoolean(print + "cone bound", encoderPosition >= IntakeConstants.CONE_MEDIUM_BOUND-1);
            }
        }
       
    }

    public void manualCloseIntake(){
        // if (!getReverseLimitSwitch() && previousLimitSwitch) { 
        //     resetEncoder();
        // }
        // previousLimitSwitch = getReverseLimitSwitch();
        if(getForwardLimitSwitch()){
            motor.set(0.0);
        }
        else
            motor.set(IntakeConstants.TARGET_VELOCITY);
    }


    // public void manualCloseIntake(double closeTo){
    //     // Update Hall effect on falling edge
    //     if (!getHallEffect() && previousLimitSwitch) { 
    //         resetEncoder();
    //     }
    //     previousLimitSwitch = getHallEffect();
    //     double encoderPosition = encoder.getPosition();
    //     if(encoderPosition > closeTo){
    //         motor.set(0.0);
    //     }
    //     else
    //         motor.set(IntakeConstants.TARGET_VELOCITY);
    // }

    // public boolean getHallEffect() {
    //     return !HallEffect.get();
    // }

    private Objects getObject() {
        currentColor = colorSensor.getColor();
        if (currentColor == null) {
            return Objects.None;
        }

        double magnitude = currentColor.blue + currentColor.red + currentColor.green;
        if (currentColor.blue / magnitude > cubeBlueTheshold)
            return Objects.Cube;
        else
            return Objects.Cone;
    }

    public boolean getForwardLimitSwitch(){
        return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public boolean getReverseLimitSwitch(){
        return motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }
    public void updateIntake() {
        SmartDashboard.putNumber("ClawPos", encoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch", getHallEffect());
        SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity());
        SmartDashboard.putBoolean("Detecting", objectInRange());
        SmartDashboard.putNumber("Red value", colorSensor.getRed());
        SmartDashboard.putNumber("Green value", colorSensor.getGreen());
        SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
        SmartDashboard.putBoolean("Hall Effect", getHallEffect());
        SmartDashboard.putString("Object", getObject().toString());
        SmartDashboard.putBoolean("Object in range", objectInRange());
        SmartDashboard.putBoolean("Reverse Limit Switch", getReverseLimitSwitch());
        SmartDashboard.putBoolean("Forward Limit Switch", getForwardLimitSwitch());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateIntake();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}