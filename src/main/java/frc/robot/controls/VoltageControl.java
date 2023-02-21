package frc.robot.controls;

public class VoltageControl {
    public static double NEO550Voltage(double torque, double radpersec, double voltageFriction){
        return 12.371 * torque + 0.01042 * radpersec + Math.signum(radpersec) * voltageFriction;
    }
    public static double GearedNEO550Voltage(double torque, double radpersec, double voltageFriction, double gearReduction){
        return NEO550Voltage(torque/gearReduction, radpersec*gearReduction, voltageFriction);
    }
    
    public static double NEOVoltage(double torque, double radpersec, double voltageFriction){
        return 4.616 * torque + 0.02015 * radpersec + Math.signum(radpersec) * voltageFriction;
    }
    public static double GearedNEOVoltage(double torque, double radpersec, double voltageFriction, double gearReduction){
        return NEOVoltage(torque/gearReduction, radpersec*gearReduction, voltageFriction);
    }


    public static double F500Voltage(double torque, double radpersec, double voltageFriction){
        return 2.56 * torque + 0.018 * radpersec + Math.signum(radpersec) * voltageFriction;
    }
    public static double GearedF500Voltage(double torque, double radpersec, double voltageFriction, double gearReduction){
        return F500Voltage(torque/gearReduction, radpersec*gearReduction, voltageFriction);
    }
}