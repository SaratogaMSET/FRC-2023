package frc.robot;
public final class Constants {
  
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }
  public final class ArmInterface{
    //Positive X Axis defined as the front face of the robot, going from back (battery) to front (roborio)
    //Positive Y axis defined as the axis perpendicular to the ground
    public static final int proximal_left_ID = 0;
    public static final int proximal_right_ID = 0;
    public static final int distal_left_ID = 0;
    public static final int distal_right_ID = 0;

    public static final boolean proximal_left_inversion = false;
    public static final boolean proximal_right_inversion = false;
    public static final boolean distal_left_inversion = false;
    public static final boolean distal_right_inversion = false;

    public static final int encoder_proximal_left_ID = 0;
    public static final int encoder_proximal_right_ID = 0;
    public static final int encoder_distal_left_ID = 0;
    public static final int encoder_distal_right_ID = 0;

    public static final double encoder_proximal_left_offset = 0;
    public static final double encoder_proximal_right_offset = 0;
    public static final double encoder_distal_left_offset = 0;
    public static final double encoder_distal_right_offset = 0;

    public static final double gear_reduction_proximal = (68.0 / 16) * (68.0 / 8) * (48.0 / 14);
    public static final double gear_reduction_distal = (68.0 / 16) * (68.0 / 8) * (48.0 / 14);

    public static final double motor_encoder_ticks_per_revolution = 2048;
  }
  public final class Arm{
      public static final int proximalMotorID = 13;
      public static final int distalMotorID = 6;

      public static final double gearProximalReduction = 45;
      public static final double gearDistalReduction = 25 * 48.0 / 32;
  }
}
