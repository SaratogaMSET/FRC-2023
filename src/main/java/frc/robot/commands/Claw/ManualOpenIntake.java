// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw.ClawIOSparkMax;
import frc.robot.subsystems.Claw.ClawSubsystem;

/** An example command that uses an example subsystem. */
public class ManualOpenIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawIOSparkMax m_intake;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualOpenIntake(ClawIOSparkMax subsystem) {
    m_intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.openIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake.getReverseLimitSwitch() || m_intake.encoder.getPosition() <= 0) 
      return true;

    return false;
  }
}
