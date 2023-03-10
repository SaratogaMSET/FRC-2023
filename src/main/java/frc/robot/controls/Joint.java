package frc.robot.controls;

public class Joint {
    protected final double L;
    protected final double m;
    protected final double I;
    protected final double CoM;
    
    public Joint(double length, double mass, double MoInertia, double CoMass) {
        this.L = length;
        this.m = mass;
        this.I = MoInertia;
        this.CoM = CoMass;
    }
}	