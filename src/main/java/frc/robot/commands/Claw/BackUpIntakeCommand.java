// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;

/** An example command that uses an example subsystem. */
public class BackUpIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawSubsystem m_intake;
  private BooleanSupplier isAuton;
  private BooleanSupplier enableAutoClose;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BackUpIntakeCommand(ClawSubsystem subsystem) {
    m_intake = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  public BackUpIntakeCommand(ClawSubsystem subsystem, BooleanSupplier isAuto, BooleanSupplier enableAutoClose){
    m_intake = subsystem;
    isAuton = isAuto;
    this.enableAutoClose = enableAutoClose;
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
      m_intake.setIdle();
    }
        if(m_intake.isGamepieceInRange()){
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
