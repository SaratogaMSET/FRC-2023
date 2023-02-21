package frc.robot.subsystems.Claw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * No ClawConfig class to include limit switch and proximity sensor since those inputs don't affect simulation
 */
public class ClawIOSim implements ClawIO {
    private SingleJointedArmSim clawSim;
    public double appliedVoltage;
    public DCMotor motor;
    public AnalogInput proximitySensor;
    public ClawConfig config;

    public ClawIOSim() {
        config = new ClawConfig();
        motor = DCMotor.getNeo550(20);
        clawSim = new SingleJointedArmSim(motor,
                Constants.IntakeConstants.GEAR_RATIO,
                0.001,
                0.2,
                0,
                Math.PI / 3,
                true);
        appliedVoltage = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ClawIOInputs inputs) {
        clawSim.update(Constants.loopPeriodSecs);
        inputs.appliedVoltage = appliedVoltage;
        inputs.rotations = clawSim.getAngleRads() / (2 * Math.PI);
        inputs.torque = ClawKinematics.getTorque();
        inputs.objectSecured = ClawKinematics.objectSecured();
        inputs.torqueBuffered = ClawKinematics.getTorqueBuffer(inputs.torque, IntakeConstants.CLOSING_TORQUE_THRESHOLD);
    }

    /** Run the motor at the specified voltage. */
    public void setMotorVoltage(double volts) {
        appliedVoltage = volts;
        clawSim.setInputVoltage(appliedVoltage);
    }

    public boolean getLimitSwitch(){
        return ClawConfig.limitSwitch.get();
    }
}
