// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.revrobotics.ColorSensorV3;
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
    boolean previousHallEffect = false;
    public static ColorSensorV3 colorSensor;

    public ClawIOSparkMax() {
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setMotorVoltage(double voltage) {
        ClawConfig.motor.setVoltage(voltage);
    }

    public void updateInputs(ClawIOInputsAutoLogged inputs) {
        inputs.rotations = ClawConfig.encoder.getPosition();
        inputs.objectDetected = objectInRange();
        inputs.proximity = proximityValue();
        inputs.object = getObject().toString();
    }

    public void openIntake() {
        updateIntake();
        double velocitySetpoint = getHallEffect()|| ClawConfig.encoder.getPosition() <= -42 ? 0.0
                : -Constants.IntakeConstants.TARGET_VELOCITY;
        SmartDashboard.putNumber("Target velocity", velocitySetpoint);
        ClawConfig.motor.set(velocitySetpoint);
    }

    public void setIdle() {
        ClawConfig.motor.set(0.0);
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

    private boolean objectInRange() {
        return proximityValue() > 65;
    }

    private void resetEncoder() {
        ClawConfig.encoder.setPosition(0);
    }

    public void autoCloseIntake(){
        if(objectInRange()){
        if (!getHallEffect() && previousHallEffect) { 
            resetEncoder();
        }
        previousHallEffect = getHallEffect();
        double encoderPosition = ClawConfig.encoder.getPosition();
            if(getObject() == Objects.Cube){
                if(encoderPosition > IntakeConstants.CUBE_MEDIUM_BOUND){
                    ClawConfig.motor.set(0.0);
                }
                else{
                    ClawConfig.motor.set(IntakeConstants.TARGET_VELOCITY);
                }
            
             }
            else if(getObject() == Objects.Cone){
                if(encoderPosition > IntakeConstants.CONE_MEDIUM_BOUND){
                    ClawConfig.motor.set(0.0);
                }
                else{
                    ClawConfig.motor.set(IntakeConstants.TARGET_VELOCITY);
                }
            }
            else{
                ClawConfig.motor.set(IntakeConstants.TARGET_VELOCITY);
            }
            SmartDashboard.putBoolean("is object cube", getObject() == Objects.Cube);
            SmartDashboard.putBoolean("cube bound", encoderPosition > IntakeConstants.CUBE_MEDIUM_BOUND);
        }
       
    }

    public void manualCloseIntake(){
        // Update Hall effect on falling edge
        if (!getHallEffect() && previousHallEffect) { 
            resetEncoder();
        }
        previousHallEffect = getHallEffect();
        double encoderPosition = ClawConfig.encoder.getPosition();
        if(encoderPosition > 50){
            ClawConfig.motor.set(0.0);
        }
        else
            ClawConfig.motor.set(0.25);
    }

    public boolean getHallEffect() {
        return !ClawConfig.HallEffect.get();
    }

    private Objects getObject() {
        currentColor = colorSensor.getColor();
        double magnitude = currentColor.blue + currentColor.red + currentColor.green;
        try {
            if (currentColor.blue / magnitude > cubeBlueTheshold)
                return Objects.Cube;
            else
                return Objects.Cone;
        } catch (NullPointerException e) {
            System.out.println("yikes");
        }
        return Objects.None;
    }

    public void updateIntake() {
        SmartDashboard.putNumber("ClawPos", ClawConfig.encoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch", getHallEffect());
        SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity());
        SmartDashboard.putBoolean("Detecting", objectInRange());
        SmartDashboard.putNumber("Red value", colorSensor.getRed());
        SmartDashboard.putNumber("Green value", colorSensor.getGreen());
        SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
        SmartDashboard.putBoolean("Hall Effect", getHallEffect());
        SmartDashboard.putString("Object", getObject().toString());
        SmartDashboard.putBoolean("Object in range", objectInRange());
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