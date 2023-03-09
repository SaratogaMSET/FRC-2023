package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.robot.Constants;


public class SwerveModuleIOSim implements SwerveModuleIO {
    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), Constants.Drivetrain.chosenModule.driveGearRatio, 0.025);
    private final FlywheelSim steerSim = new FlywheelSim(DCMotor.getFalcon500(1), Constants.Drivetrain.chosenModule.angleGearRatio, 0.004096955);
    private final PIDController drivePID = new PIDController(10, 0, 0);
    private final PIDController steerPosPID = new PIDController(15, 0, 0);

    private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double steerRelativePositionRad = steerAbsolutePositionRad;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(0.02);
        steerSim.update(0.02);

        double angleDiffRad =
                steerSim.getAngularVelocityRadPerSec() * 0.02;
        steerRelativePositionRad += angleDiffRad;
        steerAbsolutePositionRad += angleDiffRad;
        while (steerAbsolutePositionRad < 0) {
            steerAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (steerAbsolutePositionRad > 2.0 * Math.PI) {
            steerAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * Math.PI * Constants.Drivetrain.chosenModule.wheelDiameter / Constants.Drivetrain.chosenModule.driveGearRatio;
        inputs.drivePositionMeters = inputs.drivePositionMeters
                + (inputs.driveVelocityMetersPerSec * 0.02);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelcius = 0;

        inputs.steerAbsolutePositionRad = steerAbsolutePositionRad;
        inputs.steerAbsoluteVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerPositionRad = steerRelativePositionRad;
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());
        inputs.steerTempCelcius = 0;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        steerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        steerSim.setInputVoltage(steerAppliedVolts);
    }

    @Override
    public void setDesiredState(BetterSwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percent = state.speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
            driveSim.setInputVoltage(percent * 12);
        }
        else {
            drivePID.setSetpoint(state.speedMetersPerSecond);
            driveSim.setInputVoltage(drivePID.calculate(driveSim.getAngularVelocityRadPerSec() * Math.PI * Constants.Drivetrain.chosenModule.wheelDiameter / Constants.Drivetrain.chosenModule.driveGearRatio));
        }
        steerPosPID.setSetpoint(state.angle.getRadians());
        steerSim.setInputVoltage(steerPosPID.calculate(steerRelativePositionRad) + (isOpenLoop ? 0 : -0.65) * state.omegaRadPerSecond);
    }

    @Override
    public void stopMotors() {
        driveSim.setInputVoltage(0);
        steerSim.setInputVoltage(0);
    }
}