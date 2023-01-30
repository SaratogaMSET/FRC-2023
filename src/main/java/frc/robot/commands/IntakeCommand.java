// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ClawSubsystem;

// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class IntakeCommand extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ClawSubsystem m_intake;


//   private Direction direction;
//   public static enum Direction {
//     OPEN, 
//     CLOSE
//   }
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public IntakeCommand(ClawSubsystem subsystem, Direction direction) {
//     m_intake = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.direction = direction;
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     switch(direction){
//       case OPEN:
//         m_intake.openIntake();
//         break;
//       case CLOSE:
//         m_intake.closeIntake();
//         break;
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_intake.setIdle();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
