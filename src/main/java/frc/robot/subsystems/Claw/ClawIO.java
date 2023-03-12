package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.subsystems.Claw.ClawSubsystem.Objects;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        public double rotations;
        public String object;
        public boolean objectSecured;
        public boolean objectDetected;
        public double proximity;
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

    public default boolean getHallEffect() {
        return false;
    }

    public default void closeIntake(Objects object){}

    public default double closeIntake() {
        return 0.0;
    }
}
