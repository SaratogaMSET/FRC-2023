package frc.robot.subsystems.DrivetrainUtil;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIONavx implements GyroIO {
  private final AHRS gyro;
  private double prevYawPosition;
  public GyroIONavx() {
    switch (Constants.currentMode) {
      case REAL:
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIOPigeon2");
    }
  }

  public void updateInputs(GyroIOInputs inputs) {
    // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
    // "getAngle" instead of "getYaw" (what's the difference?)
    //
    // Remember to pay attention to the UNITS.
    inputs.connected = gyro.isConnected();
    inputs.yawPositionRad = -gyro.getRotation2d().getRadians();
    inputs.yawVelocityRadPerSec = (- gyro.getRotation2d().getRadians() + prevYawPosition)/0.02;
    inputs.pitchPositionRad = Math.toRadians(gyro.getPitch());
    inputs.rollPositionRad = Math.toRadians(gyro.getRoll());
    prevYawPosition = inputs.yawPositionRad;

  }
}