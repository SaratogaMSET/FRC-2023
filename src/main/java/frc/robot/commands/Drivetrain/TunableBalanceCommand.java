package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class TunableBalanceCommand extends CommandBase {

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
  private double p;
  private double d;
  private double s;
  private double v;
  private double a;

  static LoggedTunableNumber kP = new LoggedTunableNumber("Balance Kp");
  static LoggedTunableNumber kD = new LoggedTunableNumber("Balance Kd");
  static LoggedTunableNumber kS = new LoggedTunableNumber("Balance Ks");
  static LoggedTunableNumber kV = new LoggedTunableNumber("Balance Kv");
  static LoggedTunableNumber kA = new LoggedTunableNumber("Balance Ka");
  
  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public TunableBalanceCommand(DrivetrainSubsystem m_DrivetrainSubsystem) {
    this.m_DriveSubsystem = m_DrivetrainSubsystem;
    addRequirements(this.m_DriveSubsystem);
  }

  static {
    kP.initDefault(Constants.Drivetrain.balanceKP);
    kD.initDefault(Constants.Drivetrain.balanceKD);
    kS.initDefault(Constants.Drivetrain.balanceKS);
    kV.initDefault(Constants.Drivetrain.balanceKV);
    kA.initDefault(Constants.Drivetrain.balanceKA);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {

      p = kP.get();
      d = kD.get();
      s = kS.get();
      v = kV.get();
      a = kA.get();
    }

    driveRoll = new Rotation2d(m_DriveSubsystem.m_navx.getRoll());  
    drivePitch = new Rotation2d(m_DriveSubsystem.m_navx.getPitch());
    driveYaw = m_DriveSubsystem.getRotation2d();
    LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    currAcc = m_DriveSubsystem.m_navx.getWorldLinearAccelX();
    currAcc = xAccelFilter.calculate(currAcc);
    currAcc = 180 * Math.asin(currAcc/9.81)/Math.PI;
    ff = s * currAngle + v*currentAngularVelocity + a * currAcc;

    this.currentAngle = driveRoll.getRadians() * driveYaw.getSin() - drivePitch.getRadians() * driveYaw.getCos();
    
    errorDT = (error - prevError)/0.02;
    error = Constants.Drivetrain.balanceGoalDegrees - currentAngle;


    // TODO: WAS ORIGINALLY    +(Math.min(....))
    drivePower = (Math.min(p * error + d * errorDT , 1) + ff);

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
    // Debugging Print Statments
    SmartDashboard.putNumber("Current Angle: ", currentAngle);
    SmartDashboard.putNumber("Error " ,error);
    SmartDashboard.putNumber("Drive Power: " , drivePower);
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
    return Math.abs(error) <3;
  }
}