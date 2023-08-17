package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {

  private DrivetrainSubsystem m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double drivePower;
  private Rotation2d driveYaw;
  private Rotation2d drivePitch;
  private Rotation2d driveRoll;
  private double currAngle;
  private double prevError;
  private double errorDT;
  private double ff;
  private double currAcc;
  private double currentAngularVelocity;
  
  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public BalanceCommand(DrivetrainSubsystem m_DrivetrainSubsystem) {
    this.m_DriveSubsystem = m_DrivetrainSubsystem;
    addRequirements(this.m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

    driveRoll = new Rotation2d(m_DriveSubsystem.m_navx.getRoll());  
    drivePitch = new Rotation2d(m_DriveSubsystem.m_navx.getPitch());
    driveYaw = m_DriveSubsystem.getRotation2d();
    LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    currAcc = m_DriveSubsystem.m_navx.getWorldLinearAccelX();
    currAcc = xAccelFilter.calculate(currAcc);
    currAcc = 180 * Math.asin(currAcc/9.81)/Math.PI;
    ff = Constants.Drivetrain.balanceKS * currAngle + Constants.Drivetrain.balanceKV*currentAngularVelocity + Constants.Drivetrain.balanceKA * currAcc;

    this.currentAngle = driveRoll.getRadians() * driveYaw.getSin() - drivePitch.getRadians() * driveYaw.getCos();
    
    errorDT = (error - prevError)/0.02;
    error = Constants.Drivetrain.balanceGoalDegrees - currentAngle;
    

    drivePower = (Math.min(Constants.Drivetrain.balanceKP * error + Constants.Drivetrain.balanceKD * errorDT , 1) 
                  // + ff
    );

    //Robot might need an extra push when going up backwards
    if (drivePower < 0) {
      drivePower *= Constants.Drivetrain.balanceBackwardsMultiplier;
    }

    // Limit the max power
    if (Math.abs(drivePower) > 0.9) {
      drivePower = Math.copySign(0.9, drivePower);
    }
      m_DriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(0, drivePower, 0),
        driveYaw));

    prevError = error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    m_DriveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 3;
  }
}