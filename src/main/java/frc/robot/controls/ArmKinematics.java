package frc.robot.controls;
import org.ejml.simple.*;
 
public class ArmKinematics {
    public ArmKinematics() {}
    public ArmKinematics(Joint proxima, Joint distal) {
        this.proxima = proxima;
        this.distal = distal;
    }
    double x0 = 0;
    double y0 = 0;
    
 
    protected double angleProxima;
    protected double angleDistal;
    protected double omegaProxima;
    protected double omegaDistal;
    Joint proxima;
    Joint distal;

    public void setAngles(double angleProxima, double angleDistal){
        this.angleProxima = angleProxima;
        this.angleDistal = angleDistal;
    }
    public void setOmegas(double omegaProxima, double omegaDistal) {
        this.omegaProxima = omegaProxima;
        this.omegaDistal = omegaDistal;
    }
    private double sin(double x){
        return Math.sin(x);
    }
    private double cos(double x){
        return Math.cos(x);
    }
    private double angleBound(double x){
        while(x > Math.PI) x -= 2*Math.PI;
        while(x < -Math.PI) x += 2*Math.PI;
        return x;
    }
    public double[] cartesianSpeed(){
        SimpleMatrix xdot = jacobianEE().mult(new SimpleMatrix(new double[][]{{omegaProxima}, {omegaDistal}}));
        return new double[]{xdot.get(0), xdot.get(1)};
    }
    public double[] forwardKinematics(double angProxi, double angleDistall){
        double L1 = proxima.L;
        double L2 = distal.L;
        double t1 = angleProxima;
        double t2 = angleDistal;
        return new double[]{
            L1*cos(t1) + L2*cos(t2),
            L1*sin(t1) + L2*sin(t2)
        }; 
    }
    public double[] forwardKinematicsJ1(double angProxi) {
        double L1 = proxima.L;
        double t1 = angleProxima;
        return new double[]{L1*cos(t1) + x0, L1*sin(t1) + y0};
    }
    // private SimpleMatrix forwardKinematics(SimpleMatrix X) {
    //     double[] Qraw = forwardKinematics(X.get(0), X.get(1));
    //     return new SimpleMatrix(new double[][]{{Qraw[0]},{Qraw[1]}});
    // }
    public double[] regularizedTargets(double pX, double pY) {
        if(pX*pX + pY*pY > Math.hypot(proxima.L, distal.L)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (proxima.L + distal.L- 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        return new double[]{pX, pY};
    }
    public double[] inverseKinematics(double pX, double pY){
        pX -= this.x0;
        pY -= this.y0;
        //System.out.println((Math.abs(pX) < 0.01) + " "+ (-Math.signum(angleDistal)));
        double signAngle = Math.abs(pX) < 0.01 ? -Math.signum(angleDistal) : pX > 0 ? 1 : -1;
        if(pX*pX + pY*pY > Math.pow(proxima.L + distal.L, 2)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (proxima.L + distal.L - 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        double positionDistalnce = Math.hypot(pX, pY);
        //System.out.println(lenProxi + lenDistal + " " + positionDistalnce);
 
        double angleProximal = Math.atan2(pY, pX) + signAngle * Math.acos((positionDistalnce * positionDistalnce + proxima.L * proxima.L - distal.L * distal.L)/(2 * proxima.L * positionDistalnce));
        double angleDistall = signAngle * (-Math.PI + Math.acos((proxima.L * proxima.L + distal.L * distal.L - positionDistalnce * positionDistalnce)/(2 * distal.L * proxima.L)));
        angleDistall = angleBound(angleDistall + angleProximal);
        if(angleDistall > Math.PI/2) angleDistall -= 2 * Math.PI;
        return new double[]{angleBound(angleProximal), angleDistall};
    }
    // private SimpleMatrix inverseKinematics(SimpleMatrix Q) {
    //     double[] Xraw = inverseKinematics(Q.get(0), Q.get(1));
    //     return new SimpleMatrix(new double[][]{{Xraw[0]},{Xraw[1]}});
    // }
    public SimpleMatrix jacobianEE(){
        double L1 = proxima.L;
        double L2 = distal.L;
        double t1 = angleProxima;
        double t2 = angleDistal;
        double[][] cronck = {
            {-L1*sin(t1), -L2*sin(t2)},
            {L1*cos(t1), L2*cos(t2)}
        };
        SimpleMatrix jacobian = new SimpleMatrix(cronck);
        return jacobian;
    }
}