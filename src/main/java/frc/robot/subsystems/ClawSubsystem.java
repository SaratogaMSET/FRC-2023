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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ClawSubsystem extends SubsystemBase {
  private static CANSparkMax motor;
  private static RelativeEncoder encoder;
  private static DigitalInput limitSwitch;
  private double TARGET_VELOCITY;

  private double[] proximityBuffer = new double[3];
  private int bufferIndex = 0;
  private double ffVoltage = 1.5;
  private double torque = 0;
  private final double RPM_TO_RADPERSEC = 2 * Math.PI / 60;
  private double appliedVoltage;
  private double[] torqueBuffer = new double[5];
  private int torqueCounter = 0;
  private boolean objectSecuredVar;
  private double torqueThreshold = 75;

  public static ColorSensorV3 colorSensor;

  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
    // lastTime = 0.0;
    // time = 0.0;
    limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH);
    motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(360.0 / 320 * 3);
    encoder.setPosition(0);
    TARGET_VELOCITY = IntakeConstants.TARGET_VELOCITY;
    // timer = new Timer();
    objectSecuredVar = false;
    colorSensor = new ColorSensorV3(Port.kOnboard);
  }

  public void updateProximitySensor() {
    proximityBuffer[bufferIndex] = colorSensor.getProximity();
    bufferIndex++;
    bufferIndex = bufferIndex % proximityBuffer.length;
  }

  public double proximityValue() {
    double sum = 0;
    for (double x : proximityBuffer)
      sum += x;
    return sum / proximityBuffer.length;
  }

  public boolean objectInRange() {
    return proximityValue() > 35;
  }

  public void openIntake() {
    double velocitySetpoint = limitSwitch.get() ? 0.0
        : TARGET_VELOCITY;
    motor.set(-velocitySetpoint);
  }

  public void setIdle() {
    motor.set(0.0);
  }

  public void intakeVoltage(double voltage) {
    boolean lowerBound = encoder.getPosition() < 0 && Math.signum(voltage) == -1;
    boolean higherBound = encoder.getPosition() > 45 && Math.signum(voltage) == 1;

    if (!lowerBound) { //!higherBound && 
      appliedVoltage = voltage;
      motor.setVoltage(voltage);
    } else {
      motor.setVoltage(0.0);
      appliedVoltage = 0.0;
    }

    SmartDashboard.putBoolean("Upper", higherBound);
    SmartDashboard.putBoolean("Lower", lowerBound);
    SmartDashboard.putBoolean("Object Secured", objectSecured());
  }

  public boolean getTorqueBuffer(double torque) {
    torqueBuffer[torqueCounter] = torque;
    torqueCounter++;
    torqueCounter = torqueCounter % torqueBuffer.length;
    for (double tq : torqueBuffer) {
      if (tq < torqueThreshold * Math.abs(appliedVoltage / ffVoltage))
        return false;
    }
    return true;
  }

  public double getTorque(double rpm, double appliedVoltage, double gearRatio) {
    double radpersec = rpm * RPM_TO_RADPERSEC;
    SmartDashboard.putNumber("radpersec", rpm * RPM_TO_RADPERSEC);
    SmartDashboard.putNumber("rpm", rpm);
    SmartDashboard.putNumber("RPM_TO_RADPERSEC", RPM_TO_RADPERSEC);
    SmartDashboard.putNumber("applied voltage", appliedVoltage);

    torque = (Math.abs(appliedVoltage) - IntakeConstants.TORQUE_CONSTANT * Math.abs(radpersec))
        * IntakeConstants.TORQUE_CONSTANT
        / IntakeConstants.RESISTANCE;

    return torque / gearRatio;
  }

  public void closeIntake() {
    updateProximitySensor();
    objectSecuredVar = objectSecured();
    if (objectSecuredVar) {
      double holdVoltage = 0.25;
      this.appliedVoltage = holdVoltage;
      intakeVoltage(holdVoltage);
    } else if (objectInRange()) {
      intakeVoltage(ffVoltage);
    } else {
      intakeVoltage(-ffVoltage);
    }
  }

  public boolean objectSecured() {
    objectSecuredVar = getTorqueBuffer(getTorque(encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO));
    SmartDashboard.putNumber("Torque", getTorque(encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO));

    return objectSecuredVar;
  }

  public void updateObject() {
  } // Do I need to actually identify the object?

  @Override
  public void periodic() {
    updateProximitySensor();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClawPos", encoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    SmartDashboard.putNumber("Lidar (from 0-2047)", colorSensor.getProximity()); // 1-10 cm range, should be good enough
                                                                                 // for closing the close in time...
    SmartDashboard.putBoolean("Object secured", objectSecured());
    SmartDashboard.putBoolean("Detecting", objectInRange());
    SmartDashboard.putNumber("Red value", colorSensor.getRed()); // speed would have to be 45 meters per sec, so more
                                                                 // than 100 mph
    SmartDashboard.putNumber("Green value", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
    SmartDashboard.putNumber("applied voltage", appliedVoltage);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}