package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        public double rotations;
        public double torque;
        public String object;
        public boolean objectSecured;
        public boolean objectDetected;
        public double proximity;
        public double appliedVoltage;
        public boolean torqueBuffered;
        public Mechanism2d claw;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClawIOInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setMotorVoltage(double volts) {}

    /** Open the intake using torque spike */
    public default void openIntake() {}

    /**  */
    public default void setIdle() {}

    public default boolean getLimitSwitch() {
        return false;
    }

    public default double closeIntake() {
        return 0.0;
    }
}
