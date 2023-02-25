package frc.robot;
public final class Constants {
  
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public final class Arm{
      public static final int proximalMotorID = 13;
      public static final int distalMotorID = 6;

      public static final int proximalEncoderID = 13;
      public static final int distalEncoderID = 6;

      public static final double gearProximalReduction = 45;
      public static final double gearDistalReduction = 25 * 48.0 / 32;
  }
}
