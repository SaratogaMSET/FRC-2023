// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmVoltageCommand;
import frc.robot.subsystems.ArmSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driver =
      new CommandXboxController(0);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem.setDefaultCommand(
      new ArmVoltageCommand(
        armSubsystem,
        () -> modifyAxisTranslate(m_driver.getLeftY() * 1),  
        () -> modifyAxisTranslate(m_driver.getRightY() *1 )
      ));
    // Configure the trigger bindings
    configureBindings();
    
  }


//have you ever eaten a clock? it's time consuming! - toon of the sea variety
      
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driver.y().whileTrue(new ArmVoltageCommand(
    //   armSubsystem,
    //   () -> modifyAxisTranslate(m_driver.getLeftY() / 1), 
    //   () -> modifyAxisTranslate(m_driver.getRightY() / 1)));
  }

  private static double modifyAxisTranslate(double value) {

    // root the axis
    value = Math.copySign(Math.pow(value, 2), value);

    // Deadband
    value = deadband(value, 0.03);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else if (value > -deadband && value < 0) {
      return -deadband;
    } else if (value < deadband && value > 0) {
      return deadband;
    } else {
      return 0;
    }
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
