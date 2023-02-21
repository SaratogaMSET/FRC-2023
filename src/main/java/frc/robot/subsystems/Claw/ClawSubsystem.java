// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggedTunableNumber;

public class ClawSubsystem extends SubsystemBase {
  public static ColorSensorV3 colorSensor;
  public ClawIO io;
  public ClawIOVisualizer visualizer = new ClawIOVisualizer();
  public ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private static final LoggedTunableNumber limitSwitch = new LoggedTunableNumber("Claw/Limit_Switch");
  private static final LoggedTunableNumber motorVolts = new LoggedTunableNumber("Claw/Motor_Volts");
  private static final LoggedTunableNumber currentVelocity = new LoggedTunableNumber("Claw/Current_Velocity");
  private static final LoggedTunableNumber appliedTorque = new LoggedTunableNumber("Claw/Applied_Torque");

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Mechanisms/Claw", inputs);
    Logger.getInstance().recordOutput("Mechanism/Claw/Limit_Switch", io.getLimitSwitch());
    if (Constants.getMode() == Constants.Mode.REPLAY)
      visualizer.updateIntakeSim(inputs.rotations);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}