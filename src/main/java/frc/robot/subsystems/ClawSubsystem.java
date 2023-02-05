// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ClawSubsystem extends SubsystemBase {
  private static CANSparkMax motor;
  private static RelativeEncoder encoder;
  private static DigitalInput limitSwitch;
  private double TARGET_VELOCITY;

  private double[] proximityBuffer = new double[3];
  private int bufferIndex = 0;
  private double ffVoltage = 5;
  private double torque = 0;
  private final double RPM_TO_RADPERSEC = 2 * Math.PI / 60;
  private double appliedVoltage;
  private double[] torqueBuffer = new double[5];
  private int torqueCounter = 0;
  private boolean objectSecuredVar;
  private double torqueThreshold = 175;
  private double closingTorqueThreshold = 125;
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
  public ClawSubsystem() {
    limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH);
    motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(360.0 / 320 * 3);
    encoder.setPosition(0);
    TARGET_VELOCITY = IntakeConstants.TARGET_VELOCITY;
    objectSecuredVar = false;
    colorSensor = new ColorSensorV3(Port.kOnboard);
  }

  private void updateProximitySensor() {
    proximityBuffer[bufferIndex] = colorSensor.getProximity();
    bufferIndex++;
    bufferIndex = bufferIndex % proximityBuffer.length;
  }

  private double proximityValue() {
    double sum = 0;
    for (double x : proximityBuffer)
      sum += x;
    return sum / proximityBuffer.length;
  }

  private boolean objectInRange() {
    return proximityValue() > 35;
  }

  public void openIntake() {
    double velocitySetpoint = limitSwitch.get() ? 0.0
        : TARGET_VELOCITY;
    motor.set(-velocitySetpoint);
  }
  
  public void setIdle() {
    motor.set(0.0);
    appliedVoltage = 0.0;
  }

  private void resetEncoder() {
    encoder.setPosition(0.0);
  }

  public void intakeVoltage(double voltage) {
    objectState = getObject();
    if (getLimitSwitch())
      resetEncoder();
    boolean lowerBound = encoder.getPosition() < 10 && Math.signum(voltage) == -1;
    boolean coneMediumBound = encoder.getPosition() > 35 && Math.signum(voltage) == 1;
    boolean cubeMediumBound = encoder.getPosition() > 14 && Math.signum(voltage) == 1;
    boolean higherBound = encoder.getPosition() > 45 && Math.signum(voltage) == 1;

    if ((objectState == Objects.Cube && cubeMediumBound) || (objectState == Objects.Cone && coneMediumBound)) {
      appliedVoltage = 1;
    } else if ((Math.signum(voltage) == -1 && !lowerBound) || (Math.signum(voltage) == 1 && !higherBound))
      appliedVoltage = voltage;
    else
      appliedVoltage = 0.0;
    motor.setVoltage(appliedVoltage);
  }

  private boolean getTorqueBuffer(double torque, double threshold) {
    torqueBuffer[torqueCounter] = torque;
    torqueCounter++;
    torqueCounter = torqueCounter % torqueBuffer.length;
    for (double tq : torqueBuffer) {
      if (tq < threshold * Math.abs(appliedVoltage / ffVoltage))
        return false;
    }
    return true;
  }

  private double getTorque(double rpm, double appliedVoltage, double gearRatio) {
    double radpersec = rpm * RPM_TO_RADPERSEC;
    torque = (Math.abs(appliedVoltage) - IntakeConstants.TORQUE_CONSTANT * Math.abs(radpersec))
        * IntakeConstants.TORQUE_CONSTANT
        / IntakeConstants.RESISTANCE;
    return torque / gearRatio;
  }

  public void closeIntake() {
    updateProximitySensor();
    objectSecuredVar = objectSecured() && objectInRange();
    boolean objectBeingSecuredVar = objectBeingSecured() && objectInRange();
    if (objectSecuredVar) {
      double holdVoltage = 0.5;
      appliedVoltage = holdVoltage;
      intakeVoltage(holdVoltage);
    } else if (objectBeingSecuredVar) {
      appliedVoltage = 0.5;
      intakeVoltage(0.5);
    } else if (objectInRange()) {
      intakeVoltage(ffVoltage);
    } else {
      intakeVoltage(-ffVoltage);
    }
  }

  private boolean objectSecured() {
    return getTorqueBuffer(getTorque(encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO),
        torqueThreshold);
  }

  private boolean objectBeingSecured() {
    return getTorqueBuffer(getTorque(encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO),
        closingTorqueThreshold);
  }

  private boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public Objects getObject() {
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
    SmartDashboard.putNumber("ClawPos", encoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity()); // 1-10 cm range, should be good enough
                                                                                 // for closing the close in time...
    SmartDashboard.putBoolean("Detecting", objectInRange());
    SmartDashboard.putNumber("Red value", colorSensor.getRed()); // speed would have to be 45 meters per sec, so more
                                                                 // than 100 mph
    SmartDashboard.putNumber("Green value", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
    SmartDashboard.putNumber("applied voltage", appliedVoltage);
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putBoolean("Object Secured", objectSecured());
    SmartDashboard.putNumber("Torque", getTorque(encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO));
    SmartDashboard.putString("Object", objectState.toString());
    SmartDashboard.putBoolean("Object in range", objectInRange());
  }

  @Override
  public void periodic() {
    updateProximitySensor();
    // This method will be called once per scheduler run
    updateIntake();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}