// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LoggedTunableNumber;

public class ClawSubsystem extends SubsystemBase {
  public static ColorSensorV3 colorSensor;
  public ClawIO io;
  public ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  public static double TARGET_VELOCITY = 0.2;
  public static final int INTAKE_DISTANCE_THRESHOLD = 50;
  public static final double TORQUE_CONSTANT = 0.01042;
  public static final double RESISTANCE = 12 / (11000 * 2 * 3.14159265 / 60);
  public static final double TORQUE_THRESHOLD = 175;
  public static final double CLOSING_TORQUE_THRESHOLD = 125;
  private static final LoggedTunableNumber targetVelocity = new LoggedTunableNumber("Claw/Target_Velocity");

  public static enum Objects {
    Cone,
    Cube,
    None
  }

  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem(ClawIO io) {
    this.io = io;
  }

  public void openIntake() {
    io.openIntake();  
  }

  public void setIdle() {
    io.setIdle();
  }

  public void closeIntake() {
    io.closeIntake();
  }

  public void closeIntake(ClawSubsystem.Objects object){
    io.closeIntake(object);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Mechanisms/Claw", inputs);
    Logger.getInstance().recordOutput("Mechanism/Claw/Limit_Switch", io.getLimitSwitch());
    if (targetVelocity.hasChanged(hashCode())) {
      TARGET_VELOCITY = targetVelocity.get();
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}