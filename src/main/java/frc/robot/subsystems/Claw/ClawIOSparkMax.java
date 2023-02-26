// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class ClawIOSparkMax extends SubsystemBase implements ClawIO {
    private double[] proximityBuffer = new double[3];
    private int bufferIndex = 0;
    private double ffVoltage = 5;
    private double appliedVoltage;
    private boolean objectSecuredVar;
    private double cubeBlueTheshold = 0.2;
    private Objects objectState = Objects.None;
    private Color currentColor;

    public static ColorSensorV3 colorSensor;

    public static enum Objects {
        Cone,
        Cube,
        None
    }

    /** Creates a new ExampleSubsystem. */
    public ClawIOSparkMax() {
        objectSecuredVar = false;
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setMotorVoltage(double voltage){
        ClawConfig.motor.setVoltage(voltage);
    }

    public void updateInputs(ClawIOInputsAutoLogged inputs) {
        inputs.rotations = ClawConfig.encoder.getPosition() / ClawConfig.encoder.getCountsPerRevolution();
        inputs.torque = ClawKinematics.getTorque();
        inputs.appliedVoltage = ClawKinematics.appliedVoltage;
        inputs.objectSecured = ClawKinematics.objectSecured();
        inputs.objectDetected = objectInRange();
        inputs.proximity = proximityValue();
        inputs.object = getObject().toString();
        inputs.torqueBuffered = ClawKinematics.getTorqueBuffer(ClawKinematics.getTorque(), IntakeConstants.CLOSING_TORQUE_THRESHOLD);
    }

    /**
     * @return The target voltage that is needed to accurately calculate torque
     *         spikes
     */
    public double closeIntake() {
        updateIntake();
        objectSecuredVar = ClawKinematics.objectSecured() && objectInRange();
        boolean objectBeingSecuredVar = ClawKinematics.objectBeingSecured() && objectInRange();
        if (objectSecuredVar) {
            double holdVoltage = 0.5;
            ClawKinematics.setAppliedVoltage(holdVoltage);
            intakeVoltage(holdVoltage);
        } else if (objectBeingSecuredVar) {
            ClawKinematics.setAppliedVoltage(IntakeConstants.TARGET_VOLTAGE);
            intakeVoltage(IntakeConstants.TARGET_VOLTAGE);
        } else if (objectInRange()) {
            intakeVoltage(ffVoltage);
        } else {
            intakeVoltage(-ffVoltage);
        }
        return ClawKinematics.appliedVoltage;
    }

    public void openIntake() {
        double velocitySetpoint = ClawConfig.limitSwitch.get() ? 0.0
                : Constants.IntakeConstants.TARGET_VELOCITY;
        ClawConfig.motor.set(-velocitySetpoint);
    }

    public void setIdle() {
        ClawConfig.motor.set(0.0);
        ClawKinematics.setAppliedVoltage(0.0);
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
        return proximityValue() > 35;
    }

    private void resetEncoder() {
        ClawConfig.encoder.setPosition(0.0);
    }

    private void intakeVoltage(double voltage) {
        double encoderPosition = ClawConfig.encoder.getPosition();
        objectState = getObject();
        if (getLimitSwitch())
            resetEncoder();
        boolean lowerBound = encoderPosition < 2 && Math.signum(voltage) == -1; // All mutually exclusive - change into
                                                                                 // if-statements
        boolean coneMediumBound = encoderPosition > 7 && Math.signum(voltage) == 1;
        boolean cubeMediumBound = encoderPosition > 2 && Math.signum(voltage) == 1;
        boolean higherBound = encoderPosition > 12 && Math.signum(voltage) == 1;
        SmartDashboard.putBoolean("Higher bound", higherBound); // speed would
        // SmartDashboard.putBoolean("Higher bound", higherBound);

        /*   if(higherBound)
            appliedVoltage = 0;
        else  */
     if ((objectState == Objects.Cube && cubeMediumBound) || (objectState == Objects.Cone && coneMediumBound)) {
            appliedVoltage = 1;
        } else if ((Math.signum(voltage) == -1 && !lowerBound) || (Math.signum(voltage) == 1 && !higherBound))
            appliedVoltage = voltage;
        else
            appliedVoltage = 0.0;
        ClawKinematics.setAppliedVoltage(appliedVoltage);
        ClawConfig.motor.setVoltage(appliedVoltage);
    }

    public boolean getLimitSwitch() {
        return ClawConfig.limitSwitch.get();
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
        }
        return Objects.None;
    }

    public void updateIntake() {
        Logger.getInstance().recordOutput("Mechanism/Claw/Claw_torque", ClawKinematics.getTorque(ClawConfig.encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO));
    SmartDashboard.putNumber("ClawPos", ClawConfig.encoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", ClawConfig.limitSwitch.get());
    SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity());
    // 1-10 cm range, should be good enough
    // for closing the close in time...
    SmartDashboard.putBoolean("Detecting", objectInRange());
    SmartDashboard.putNumber("Red value", colorSensor.getRed()); // speed would
    // have to be 45 meters per sec, so more
    // than 100 mph
    SmartDashboard.putNumber("Green value", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
    SmartDashboard.putNumber("applied voltage", ClawKinematics.appliedVoltage);
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putBoolean("Object Secured", ClawKinematics.objectSecured());
    SmartDashboard.putNumber("Torque", ClawKinematics.getTorque(ClawConfig.encoder.getVelocity(),
    ClawKinematics.appliedVoltage, IntakeConstants.GEAR_RATIO));
    SmartDashboard.putString("Object", objectState.toString());
    SmartDashboard.putBoolean("Object in range", objectInRange());
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        // updateIntake();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}