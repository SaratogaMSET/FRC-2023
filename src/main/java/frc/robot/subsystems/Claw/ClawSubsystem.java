// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ClawSubsystem 
    extends SubsystemBase{
        
    private double[] proximityBuffer = new double[3];
    private int bufferIndex = 0;
    private double cubeBlueTheshold = 0.22;
    private Color currentColor;
    public boolean startedAutoClose = false;
    public ColorSensorV3 colorSensor;
    public CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    public RelativeEncoder encoder = motor.getEncoder();
    String print = "Claw/"; 

    public static enum GamePiece {
        Cone,
        Cube,
        None
    }

    public ClawSubsystem() {
        motor.setIdleMode(IdleMode.kBrake);
        colorSensor = new ColorSensorV3(Port.kOnboard);
        motor.setControlFramePeriodMs(30);
    }

    // public void updateInputs(ClawIOInputsAutoLogged inputs) {
    //     inputs.rotations = encoder.getPosition();
    //     inputs.objectDetected = isGamepieceInRange();
    //     inputs.proximity = getProximityValue();
    //     inputs.object = getGamePieceType().toString();
    // }
    
    public void setBrakeMode(){
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void openClaw() {
        updateClawTelemetry();
        double closeVelocity = 0.0;
        if(isClawFullyOpen()) {
            closeVelocity = 0.0;
            resetEncoder();
        }
        else{ 
            closeVelocity = -0.6;
        }
        motor.set(closeVelocity);
    }

    public void setIdle() {
        motor.set(0.0);
        startedAutoClose = false;
    }

    private double getProximityValue() {
        proximityBuffer[bufferIndex] = colorSensor.getProximity();
        bufferIndex++;
        bufferIndex = bufferIndex % proximityBuffer.length;

        double sum = 0;
        for (double x : proximityBuffer)
            sum += x;
        return sum / proximityBuffer.length;
    }

    public boolean isGamepieceInRange() {
        return getProximityValue() > 65;
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    public void autoCloseClaw() {
        if(isGamepieceInRange()) {
            if (isClawFullyOpen() || startedAutoClose) {
                // Either the claw is fully open or we had previously started auto close.
                startedAutoClose = true;
                double encoderPosition = encoder.getPosition();
                if (getGamePieceType() == GamePiece.Cube) {
                    if(encoderPosition >= IntakeConstants.CUBE_MEDIUM_BOUND - 1) {
                        motor.set(0.0);
                    } else {
                        motor.set(IntakeConstants.TARGET_VELOCITY);
                    }
                
                } else if (getGamePieceType() == GamePiece.Cone) {
                    if (encoderPosition >= IntakeConstants.CONE_MEDIUM_BOUND - 1) {
                        motor.set(0.0);
                    } else {
                        motor.set(IntakeConstants.TARGET_VELOCITY);
                    }
                } else {
                    motor.set(0.0);
                }
            }
        }
    }

    public void manualCloseClaw() {
        if (isClawFullyClosed()) {
            motor.set(0.0);
        } else {
            motor.set(IntakeConstants.TARGET_VELOCITY);
        }
    }

    private GamePiece getGamePieceType() {
        currentColor = colorSensor.getColor();
        if (currentColor == null) {
            return GamePiece.None;
        }

        double magnitude = currentColor.blue + currentColor.red + currentColor.green;
        if (currentColor.blue / magnitude > cubeBlueTheshold)
            return GamePiece.Cube;
        else
            return GamePiece.Cone;
    }

    private boolean isClawFullyClosed() {
        return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public boolean isClawFullyOpen() {
        return motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public void updateClawTelemetry() {
        SmartDashboard.putNumber("ClawPos", encoder.getPosition());
        // SmartDashboard.putBoolean("Limit Switch", getHallEffect());
        SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity());
        // SmartDashboard.putBoolean("Detecting", objectInRange());
        SmartDashboard.putNumber("Red value", colorSensor.getRed());
        SmartDashboard.putNumber("Green value", colorSensor.getGreen());
        SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
        // SmartDashboard.putBoolean("Hall Effect", getHallEffect());
        SmartDashboard.putString("Object", getGamePieceType().toString());
        // SmartDashboard.putBoolean("Object in range", objectInRange());
        SmartDashboard.putBoolean("Reverse Limit Switch", isClawFullyOpen());
        SmartDashboard.putBoolean("Forward Limit Switch", isClawFullyClosed());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateClawTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}