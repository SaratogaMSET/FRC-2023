package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggedTunableNumber;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {
  private final DrivetrainSubsystem drive;
  private Rotation2d driveYaw;
  private Rotation2d drivePitch;
  private Rotation2d driveRoll;

    @Override
    public void initialize(){
        intitalPitch = drivetrain.m_navx.getRoll();
    }
  private static final double AutoBalanceKP = 0.5;
    //   new LoggedTunableNumber("ChargeStationAutoBalance/AutoBalanceKP");
  private static final double AutoBalanceKI = 0.0;
    //   new LoggedTunableNumber("ChargeStationAutoBalance/AutoBalanceKI");
  private static final double AutoBalanceKD = 0.0;
    //   new LoggedTunableNumber("ChargeStationAutoBalance/AutoBalanceKD");
  double driveSpeedMetersPerSec = 5.0;

    @Override
    public void execute(){
        
    }
  private final PIDController pidController =
      new PIDController(AutoBalanceKP, AutoBalanceKI, AutoBalanceKD);

  /** Creates a new ChargeStationAutoBalance. */
  public BalanceCommand(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update tunnable numbers if changed
    // if (AutoBalanceKD.hasChanged(hashCode())
    //     || AutoBalanceKI.hasChanged(hashCode())
    //     || AutoBalanceKP.hasChanged(hashCode())) {
    //   pidController.setP(AutoBalanceKP.get());
    //   pidController.setI(AutoBalanceKI.get());
    //   pidController.setD(AutoBalanceKD.get());
    // }

    @Override
    public void end(boolean interrupted){
    // get rotation of robot
    driveRoll = new Rotation2d(drive.m_navx.getRoll());
    drivePitch = new Rotation2d(drive.m_navx.getPitch());
    driveYaw = drive.getRotation2d();

    double chargeStationAngle =
        driveRoll.getRadians() * driveYaw.getSin() + drivePitch.getRadians() * driveYaw.getCos();

    // P controller
    double driveSpeedAfterPID = pidController.calculate(chargeStationAngle);

    // charge station limits
    double chargeStationInnerX = DrivetrainSubsystem.apply(Units.inchesToMeters(193.25) - Units.inchesToMeters(2.0) - (Units.inchesToMeters(76.125)/2)); //chargingStationInnerX
    double chargeStationOuterX = DrivetrainSubsystem.apply(Units.inchesToMeters(193.25) - Units.inchesToMeters(2.0)); //chargingStationOuterX
    
    // speed limits depending on alliance
    if (DriverStation.getAlliance() == Alliance.Blue) {
      // check boundaries for left of charge station
      if (drive.getPose().getX() <= chargeStationInnerX && driveSpeedAfterPID < 0) {
        driveSpeedAfterPID = 0;
      }

      // check boundaries for right of charge station
      if (drive.getPose().getX() > chargeStationOuterX
          && driveSpeedAfterPID > 0) {
        driveSpeedAfterPID = 0;
      }
    }else {
      // check boundaries for left of charge station
      if (drive.getPose().getX() <= chargeStationInnerX && driveSpeedAfterPID > 0) 
      {
        driveSpeedAfterPID = 0;
      }

      // check boundaries for right of charge station
      if (drive.getPose().getX() > chargeStationOuterX
          && driveSpeedAfterPID < 0) {
        driveSpeedAfterPID = 0;
      }
    }

    // command drive subsystem
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(driveSpeedAfterPID, 0, 0),driveYaw));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.mSwerveMods[3].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 1/4),0.0));
    drive.mSwerveMods[2].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 3/4),0.0));

    drive.mSwerveMods[1].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 3/4),0.0));
    drive.mSwerveMods[0].setAngle(new BetterSwerveModuleState(0.0, new Rotation2d(Math.PI * 1/4),0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}