// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  // private static CANSparkMax m_intake;
  private static RelativeEncoder encoder;
  // private static DigitalInput limitSwitch;
  // private LidarSubsystem lidar_1;
  private double TARGET_VELOCITY;
  private double lastTime;
  private double time;
  private Timer timer;
  

  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
    // limitSwitch = new DigitalInput(Constants.IntakeConstants.LIMIT_SWITCH);
    // m_intake = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    // m_intake.setIdleMode(IdleMode.kBrake);
    // encoder = m_intake.getEncoder();
    TARGET_VELOCITY = Constants.IntakeConstants.TARGET_VELOCITY;
    timer = new Timer();
    // lidar_1 = new LidarSubsystem(0);
  }

  public void closeIntake() {
    time = (double) timer.get();
    double velocity = encoder.getVelocity();
    // double pulledVolt = m_intake.getBusVoltage();
    double acceleration = velocity / (lastTime - time);
    double velocitySetpoint = Math.abs(velocity) < 0.05 && acceleration < 0.0 ? 0.0
        : TARGET_VELOCITY; 
    // m_intake.set(velocitySetpoint);
    lastTime = time;
}

  public void openIntake() {
    // double velocitySetpoint = limitSwitch.get() ? 0.0
    //     : TARGET_VELOCITY;
    // m_intake.set(-velocitySetpoint);
  }

  public void setIdle(){
    // m_intake.set(0.0);
  }

  public void updateLidar(){
    // SmartDashboard.putNumber("Lidar", lidar_1.getDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}