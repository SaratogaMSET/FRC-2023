// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw.ClawSubsystem;

/**
 * Default intake command
 */
public class DefaultIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawSubsystem m_intake;
  private BooleanSupplier isAuton;
  private BooleanSupplier enableAutoClose;
  /**
   * Creates a new DefaultIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultIntakeCommand(ClawSubsystem subsystem) {
    m_intake = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  public DefaultIntakeCommand(ClawSubsystem subsystem, BooleanSupplier isAuto, BooleanSupplier enableAutoClose){
    m_intake = subsystem;
    isAuton = isAuto;
    this.enableAutoClose = enableAutoClose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isAuton.getAsBoolean()){  // If not in autonomous mode, set intake to idle mode
      m_intake.setIdle();
    }
        if(m_intake.isGamepieceInRange()){  // Run auto close if it is enabled and game piece is in range
            if(enableAutoClose.getAsBoolean()){
                m_intake.autoCloseClaw();
        }
    }
    else{
      m_intake.setIdle();
    }
  // }
    m_intake.setIdle();
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
