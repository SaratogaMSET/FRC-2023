package frc.robot.subsystems.Claw;

import frc.robot.Constants.IntakeConstants;

public class ClawKinematics {
    // private static double[] proximityBuffer = new double[3];
    // private static int bufferIndex = 0;
    private static double ffVoltage = 5;
    private static double torque = 0;
    private static final double RPM_TO_RADPERSEC = 2 * Math.PI / 60;
    public static double appliedVoltage;
    private static double[] torqueBuffer = new double[5];
    private static int torqueCounter = 0;

    public static boolean getTorqueBuffer(double torque, double threshold) {
        torqueBuffer[torqueCounter] = torque;
        torqueCounter++;
        torqueCounter = torqueCounter % torqueBuffer.length;
        for (double tq : torqueBuffer) {
            if (tq < threshold * Math.abs(appliedVoltage / ffVoltage))
                return false;
        }
        return true;
    }

    public static void setAppliedVoltage(double newAppliedVoltage){
        appliedVoltage = newAppliedVoltage;
    }

    public static double getAppliedVoltage(){
        return appliedVoltage;
    }

    public static double getTorque() {
        return getTorque(ClawConfig.encoder.getVelocity(), appliedVoltage, IntakeConstants.GEAR_RATIO);
    }

    public static double getTorque(double rpm, double appliedVoltage, double gearRatio) {
        double radpersec = rpm * RPM_TO_RADPERSEC;
        torque = (Math.abs(appliedVoltage) - IntakeConstants.TORQUE_CONSTANT * Math.abs(radpersec))
                * IntakeConstants.TORQUE_CONSTANT
                / IntakeConstants.RESISTANCE;
        return torque / gearRatio;
    }

    public static boolean objectSecured() {
        return ClawKinematics.getTorqueBuffer(ClawKinematics.getTorque(), IntakeConstants.TORQUE_THRESHOLD);
    }

    public static boolean objectBeingSecured() {
        return ClawKinematics.getTorqueBuffer(ClawKinematics.getTorque(), IntakeConstants.CLOSING_TORQUE_THRESHOLD);
    }
}
