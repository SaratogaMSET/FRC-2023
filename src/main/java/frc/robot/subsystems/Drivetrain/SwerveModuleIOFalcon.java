package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModuleIOFalcon implements SwerveModuleIONew {
  private int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX steerMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Drivetrain.driveKS, Constants.Drivetrain.driveKV, Constants.Drivetrain.driveKA);

  public SwerveModuleIOFalcon(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    steerMotor = new TalonFX(moduleConstants.angleMotorID);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor();
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.moduleNumber = moduleNumber;

    inputs.drivePositionRotations = driveMotor.getSelectedSensorPosition() / 2048;
    inputs.driveSpeedRPS = driveMotor.getSelectedSensorVelocity() * 10 / 2048;
    inputs.drivePercentOut = driveMotor.getMotorOutputPercent();
    inputs.driveCurrentAmps = driveMotor.getStatorCurrent();
    inputs.driveTemparature = driveMotor.getTemperature();

    inputs.absoluteEncoderRotations = getAbsoluteRotation().getRadians();

    inputs.steerPositionRotations = steerMotor.getSelectedSensorPosition() / 2048;
    inputs.steerSpeedRPS = steerMotor.getSelectedSensorVelocity() * 10 / 2048;
    inputs.steerPercentOut = steerMotor.getMotorOutputPercent();
    inputs.steerCurrentAmps = steerMotor.getStatorCurrent();
    inputs.steerTemparature = steerMotor.getTemperature();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              Constants.Drivetrain.wheelCircumference,
              Constants.Drivetrain.driveGearRatio);
      driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
            ? getAngle()
            : desiredState.angle;

    steerMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Drivetrain.angleGearRatio));
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            steerMotor.getSelectedSensorPosition(), Constants.Drivetrain.angleGearRatio));
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  @Override
  public void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getAbsoluteRotation().getDegrees() - angleOffset.getDegrees(),
            Constants.Drivetrain.angleGearRatio);
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    steerMotor.setInverted(Constants.Drivetrain.angleMotorInvert);
    steerMotor.setNeutralMode(Constants.Drivetrain.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(Constants.Drivetrain.driveMotorInvert);
    driveMotor.setNeutralMode(Constants.Drivetrain.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            Constants.Drivetrain.wheelCircumference,
            Constants.Drivetrain.driveGearRatio),
        getAngle());
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            Constants.Drivetrain.wheelCircumference,
            Constants.Drivetrain.driveGearRatio),
        getAngle());
  }

  @Override
  public int getModuleNumber() {
    return moduleNumber;
  }
}