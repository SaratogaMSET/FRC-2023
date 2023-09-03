package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
@Deprecated
public interface GyroIO {
  
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPositionRad = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double pitchPositionRad = 0.0;
    public double rollPositionRad = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}