// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.Direction;
import frc.robot.subsystems.Claw.ClawIOSparkMax;
import frc.robot.subsystems.Claw.ClawSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private ClawIOSparkMax m_claw;
  // private IntakeSubsystemWheel m_intakeSubsystem;
  private XboxController m_controller;
  private TalonFX motor;
  // private Logger logger;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor = new TalonFX(54);
    // logger = Logger.getInstance();
    // logger.recordMetadata("Claw_Prototype", "Claw_Prototype"); // Set a metadata value
    // switch (Constants.getMode()) {
    //   case REAL:
    //     String folder = "/Users/User/Documents/FRC-2023-Logs";
    //     if (folder != null) {
    //       logger.addDataReceiver(new WPILOGWriter(folder));
    //     }
    //     LoggedPowerDistribution.getInstance();
    //   case REPLAY:
    //     setUseTiming(false); // Run as fast as possible
    //     String path = LogFileUtil.findReplayLog();
    //     logger.setReplaySource(new WPILOGReader(path));
    //     logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path,
    //         "_sim")));
    //     break;
    //   case NOT_SET:
    //     break;
    // }
    // logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    m_claw = new ClawIOSparkMax();
    // m_intakeSubsystem = new IntakeSubsystemWheel();
    m_controller = new XboxController(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_intakeSubsystem.runIntake(m_controller.getLeftY());
    m_claw.closeIntake();
    // new Trigger(m_controller::getRightBumper).whileTrue(new IntakeCommand(m_claw, Direction.CLOSE));
    // new Trigger(m_controller::getRightBumper).whileFalse(new IntakeCommand(m_claw, Direction.OPEN));

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    motor.set(ControlMode.PercentOutput, 0.5);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
