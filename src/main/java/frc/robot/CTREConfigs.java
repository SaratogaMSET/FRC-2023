package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.angleEnableCurrentLimit, 
            Constants.Drivetrain.angleContinuousCurrentLimit, 
            Constants.Drivetrain.anglePeakCurrentLimit, 
            Constants.Drivetrain.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Drivetrain.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Drivetrain.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Drivetrain.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Drivetrain.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.driveEnableCurrentLimit, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            Constants.Drivetrain.drivePeakCurrentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Drivetrain.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Drivetrain.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Drivetrain.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Drivetrain.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Drivetrain.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Drivetrain.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Drivetrain.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}