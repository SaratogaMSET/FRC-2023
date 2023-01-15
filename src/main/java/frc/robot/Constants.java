package frc.robot;
public final class Constants {
  public final class Arm{
      public static final int proximalMotorID = 13;
      public static final int distalMotorID = 6;

      public static final int proximalEncoderID = 3;
      public static final int distalEncoderID = 4;

      public static final double gearProximalReduction = 45;
      public static final double gearDistalReduction = 50;

      public static final double initialAngleProximal = Math.PI / 180 * (150);
      public static final double initialAngleDistal = Math.PI / 180 * (-170);
  }
}
