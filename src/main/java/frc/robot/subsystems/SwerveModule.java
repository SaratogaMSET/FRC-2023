package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainUtil.SwerveModuleIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule implements SwerveModuleIO {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.driveKS, Constants.Drivetrain.driveKV, Constants.Drivetrain.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

        
    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */

        desiredState = CTREModuleState.optimize(desiredState, getState().angle,0); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    public void setDesiredState(SwerveModuleState swerveModuleState, boolean isOpenLoop) {
        swerveModuleState = CTREModuleState.optimize(swerveModuleState, getState().angle); 
        setAngle(swerveModuleState);
        setSpeed(swerveModuleState, isOpenLoop);
    }
        private void setSpeed(BetterSwerveModuleState desiredState, boolean isOpenLoop){
            if(isOpenLoop){
                double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
                mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            }
            else {
                double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Drivetrain.wheelCircumference, Constants.Drivetrain.driveGearRatio);
                mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            
        }
    }
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Drivetrain.wheelCircumference, Constants.Drivetrain.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        
    }
    }

    public void setAngle(BetterSwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Drivetrain.angleGearRatio));
        lastAngle = angle;
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Drivetrain.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Drivetrain.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }
    public Rotation2d getOffset(){
        return angleOffset;
    }
    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Drivetrain.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Drivetrain.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Drivetrain.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Drivetrain.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Drivetrain.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Drivetrain.wheelCircumference, Constants.Drivetrain.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Drivetrain.wheelCircumference, Constants.Drivetrain.driveGearRatio), 
            getAngle()
        );
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        
        inputs.drivePositionMeters = mDriveMotor.getSelectedSensorVelocity();
        inputs.driveVelocityMetersPerSec = mDriveMotor.getSelectedSensorVelocity();
        inputs.driveAppliedVolts = mDriveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = mDriveMotor.getStatorCurrent();
        inputs.driveTempCelcius = mDriveMotor.getTemperature();

        inputs.steerAbsolutePositionRad = angleEncoder.getPosition();
        inputs.steerAbsoluteVelocityRadPerSec = angleEncoder.getVelocity();
        inputs.steerPositionRad = mAngleMotor.getSelectedSensorVelocity();
        inputs.steerVelocityRadPerSec = mAngleMotor.getSelectedSensorVelocity();
        inputs.steerAppliedVolts = mAngleMotor.getMotorOutputVoltage();
        inputs.steerCurrentAmps = mAngleMotor.getStatorCurrent();
        inputs.steerTempCelcius = mAngleMotor.getTemperature();
    }


    

}