package frc.robot;

import frc.robot.controls.Joint;

public final class Constants {
  
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }
  public final class ArmParameters{
    //Positive X Axis defined as the front face of the robot, going from back (battery) to front (roborio)
    //Positive Y axis defined as the axis perpendicular to the ground
    public static final int proximal_left_ID = 23;
    public static final int proximal_right_ID = 52;
    public static final int distal_left_ID = 39;
    public static final int distal_right_ID = 50;

    public static final boolean proximal_left_inversion = false;
    public static final boolean proximal_right_inversion = true;
    public static final boolean distal_left_inversion = false;
    public static final boolean distal_right_inversion = true;

    public static final int encoder_proximal_left_ID = 12;
    public static final int encoder_proximal_right_ID = 11;
    public static final int encoder_distal_left_ID = 13;
    public static final int encoder_distal_right_ID = 10;

    public static final double encoder_proximal_left_offset = 0.775;
    public static final double encoder_proximal_right_offset = 0.25;
    public static final double encoder_distal_left_offset = 0.511;
    public static final double encoder_distal_right_offset = 0.759;

    public static final double gear_reduction_proximal = (68.0 / 16) * (68.0 / 8) * (48.0 / 14);
    public static final double gear_reduction_distal = (68.0 / 16) * (68.0 / 8) * (48.0 / 16);

    public static final double motor_encoder_ticks_per_revolution = 2048;

    public static final double proximal_highbound = 140 * (Math.PI / 180);
    public static final double proximal_lowbound = 40 * (Math.PI / 180);
    public static final double distal_highbound = (20) * (Math.PI / 180);
    public static final double distal_lowbound = (- 180 - 20) * (Math.PI / 180);

    public static final double pounds_to_kilograms = 0.453592;
    public static final double inches_to_meters = 0.0254;
    public static final double lbs_sqinches_to_kg_sqmeters = pounds_to_kilograms * inches_to_meters * inches_to_meters;
    
    public static final double proximal_length = 40 * inches_to_meters;
    public static final double proximal_mass = 4*2 *pounds_to_kilograms;
    public static final double proximal_inertia = 2*2961.95 * lbs_sqinches_to_kg_sqmeters;
    public static final double proximal_com = 22.80 * inches_to_meters;

    public static final double distal_length = 33 * inches_to_meters;
    public static final double distal_mass = 2.8*2 * pounds_to_kilograms;
    public static final double distal_inertia = 2*866.840 * lbs_sqinches_to_kg_sqmeters;
    public static final double distal_com = 13.56 * inches_to_meters;
  }
}
