// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;
  // private ClawIOSparkMax clawSubsystem = new ClawIOSparkMax();
  // private IntakeSubsystemWheel m_intakeSubsystem;
  // private XboxController m_controller;
  // private TalonFX motor;
  // private Logger logger;
  // private final ClawSubsystem m_claw = new ClawSubsystem(clawSubsystem);
  public Robot(){
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.getInstance().recordMetadata("FRC-2023", "FRC-2023"); // Set a metadata value
    switch (Constants.currentMode) {
      case REAL:
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;

      case SIM:
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(path));
        Logger.getInstance().addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_claw.setBrakeMode();
    // LiveWindow.enableAllTelemetry();
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
    m_robotContainer.updateRobotState();
    // m_claw.closeIntake();
    CommandScheduler.getInstance().run();

    if(
      RobotContainer.m_driverController.povDown().getAsBoolean() ||
      RobotContainer.m_driverController.povDownLeft().getAsBoolean() ||
      RobotContainer.m_driverController.povDownRight().getAsBoolean() ||
      RobotContainer.m_driverController.povLeft().getAsBoolean() ||
      RobotContainer.m_driverController.povRight().getAsBoolean() ||
      RobotContainer.m_driverController.povUp().getAsBoolean() ||
      RobotContainer.m_driverController.povUpLeft().getAsBoolean() ||
      RobotContainer.m_driverController.povUpRight().getAsBoolean()){

        CommandScheduler.getInstance().cancelAll();
      // CommandScheduler.getInstance().close();
    }
    // m_intakeSubsystem.runIntake(m_controller.getLeftY());
  
    // new Trigger(m_controller::getRightBumper).whileTrue(new IntakeCommand(m_claw, Direction.CLOSE));
    // new Trigger(m_controller::getRightBumper).whileFalse(new IntakeCommand(m_claw, Direction.OPEN));
    // SmartDashboard.putNumber("Proximity", ClawIOSparkMax.colorSensor.getProximity());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().close();
    m_robotContainer.m_claw.setCoastMode();
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
    m_robotContainer.m_claw.setBrakeMode();
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
    m_robotContainer.m_claw.setBrakeMode();
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
    // m_claw.closeIntake();
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
