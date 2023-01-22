// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// @Deprecated
public class IntakeSubsystemWheel extends SubsystemBase {
  private CANSparkMax m_intake;
  private static DigitalInput limitSwitch;
  private static double INTAKE_SPEED = 0.0;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystemWheel() {
    m_intake = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    m_intake.setIdleMode(IdleMode.kBrake);
    limitSwitch = new DigitalInput(Constants.IntakeConstants.LIMIT_SWITCH);
  }
  public static enum Direction {
    INTAKE,
    OUTTAKE,
    IDLE
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void runIntake(Direction direction) {
    INTAKE_SPEED = 0.5;  //
    if (limitSwitch.get()) {  // default limit switch state
      switch (direction) {
        case INTAKE:
          m_intake.set(INTAKE_SPEED);
          break;
        case OUTTAKE:
          m_intake.set(-0.1);
          break;
        case IDLE:
          m_intake.set(0.0);
          break;
        default:
          SmartDashboard.putNumber("Intake Speed", 649);
      }
    }
  }

  public boolean updateIntakeState() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateIntakeState();
    SmartDashboard.putBoolean("LS", updateIntakeState());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}