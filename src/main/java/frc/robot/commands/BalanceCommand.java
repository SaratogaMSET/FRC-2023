package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {

  private DrivetrainSubsystem m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public BalanceCommand(DrivetrainSubsystem m_DrivetrainSubsystem) {
    this.m_DriveSubsystem = m_DrivetrainSubsystem;
    addRequirements(this.m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = m_DriveSubsystem.m_navx.getPitch();

    error = Constants.Drivetrain.balanceGoalDegrees - currentAngle;
    drivePower = -Math.min(Constants.Drivetrain.balanceKP * error, 1);

    //Robot might need an extra push when going up backwards
    if (drivePower < 0) {
      drivePower *= Constants.Drivetrain.balanceBackwardsMultiplier;
    }

    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    m_DriveSubsystem.drive(new ChassisSpeeds()); //ChassisSpeeds with sign
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.Drivetrain.balanceDriveTurningDegrees; 
  }
}