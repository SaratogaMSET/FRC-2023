// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw.ClawIOSparkMax;
import frc.robot.subsystems.Claw.ClawSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawIOSparkMax m_intake;
  private BooleanSupplier isAuton;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(ClawIOSparkMax subsystem) {
    m_intake = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  public IntakeCommand(ClawIOSparkMax subsystem, BooleanSupplier isAuto){
    m_intake = subsystem;
    isAuton = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // switch(direction){
    //   case OPEN:
    //     m_intake.openIntake();
    //     break;
    //   case CLOSE:
    //     m_intake.closeIntake();
    //     break;
    //   case IDLE:
    //     m_intake.setIdle();
    //     break;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isAuton.getAsBoolean()){
      if(m_intake.objectInRange()){
        m_intake.autoCloseIntake();
      }
    }
    else{
      m_intake.setIdle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
