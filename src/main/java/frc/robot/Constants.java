package frc.robot;
public final class Constants {
  public final class Arm{
      public static final int proximalMotorID = 13;
      public static final int distalMotorID = 6;

      public static final int proximalEncoderID = 13;
      public static final int distalEncoderID = 6;

      public static final double gearProximalReduction = 45 * 77 / 90.0;
      public static final double gearDistalReduction = 50 * 127 / 180.0;

      public static final double initialAngleProximal = 90; //Degr
      public static final double initialAngleDistal = -90;
  }
}
