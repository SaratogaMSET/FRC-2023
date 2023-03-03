// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR = 6;
    public static final int WHEEL_INTAKE_MOTOR = 7;
    public static final double GEAR_RATIO = 3.0 / 320.0;
    public static final int LIMIT_SWITCH = 0;
    public static final double TARGET_VELOCITY = 0.2;
    public static final int INTAKE_DISTANCE_THRESHOLD = 50;
    public static final double TORQUE_CONSTANT = 0.01042;
    public static final double RESISTANCE = 12 / (11000 * 2 * 3.14159265 / 60);
    public static final double TARGET_VOLTAGE = 5.0;
    public static final double TORQUE_THRESHOLD = 125;
    public static final double CLOSING_TORQUE_THRESHOLD = 75;
    public static final double LOWER_BOUND = 0.5;
    public static final double CUBE_MEDIUM_BOUND = 2;
    public static final double CONE_MEDIUM_BOUND = 7;
    public static final double UPPER_BOUND = 12;
    public static final double HOLD_VOLTAGE = TARGET_VOLTAGE / 10;
    public static final double AUTO_CONE_DIAMETER = 0.0;
    public static final double AUTO_CUBE_DIAMETER = 0.0;
    public static final double AUTO_KP = 0.0;
    public static final double AUTO_KI = 0.0;
    public static final double AUTO_KD = 0.0;
  }

  public static Mode getMode() {
      return Mode.REAL;
  }

  public static enum Mode {
    REAL,
    REPLAY,
    NOT_SET
  }
}
