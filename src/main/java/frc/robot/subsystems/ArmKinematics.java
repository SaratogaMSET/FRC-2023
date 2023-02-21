package frc.robot.subsystems;
import org.ejml.simple.*;
 
public class ArmKinematics {
    public ArmKinematics() {}
    public ArmKinematics(double initX, double initY) {
        this.x0 = initX;
        this.y0 = initY;
    }
    double x0 = 0;
    double y0 = 0;
    
    double boundsProxima[] = {Math.toDegrees(30), Math.toDegrees(150)};
    double boundsDista[] = {Math.toDegrees(-150), Math.toDegrees(150)};
 
    protected double angleProxima;
    protected double angleDista;
    protected double omegaProxima;
    protected double omegaDista;
    Joint proxima = new Joint(0.4445, 0.573 + 0.210, 0.3);
    Joint dista = new Joint(0.3683, 0.488 + 0.210, 0.3);
 
    public class Joint {
        public Joint(double length, double mass, double CenterOfMass){
            this.length = length;
            this.mass = mass;
            this.CoM = CenterOfMass;
            this.MoI = mass * CenterOfMass * CenterOfMass;
        }
        protected double length; //Meters
        protected double CoM; //Meters (Distance of center of mass from center)
        protected double mass; //Kilograms
        protected double MoI; // kg m^2
    }
    public void setAngles(double angleProxima, double angleDista){
        this.angleProxima = angleProxima;
        this.angleDista = angleDista;
    }
    public void setOmegas(double omegaProxima, double omegaDista) {
        this.omegaProxima = omegaProxima;
        this.omegaDista = omegaDista;
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
        SimpleMatrix xdot = jacobianEE().mult(new SimpleMatrix(new double[][]{{omegaProxima}, {omegaDista}}));
        return new double[]{xdot.get(0), xdot.get(1)};
    }
    public double[] forwardKinematics(double angProxi, double angleDistal){
        double L1 = proxima.length;
        double L2 = dista.length;
        double t1 = angleProxima;
        double t2 = angleDista;
        return new double[]{
            L1*cos(t1) + L2*cos(t2),
            L1*sin(t1) + L2*sin(t2)
        }; 
    }
    public double[] forwardKinematicsJ1(double angProxi) {
        double L1 = proxima.length;
        double t1 = angleProxima;
        return new double[]{L1*cos(t1) + x0, L1*sin(t1) + y0};
    }
    private SimpleMatrix forwardKinematics(SimpleMatrix Q) {
        double[] Xraw = forwardKinematics(Q.get(0), Q.get(1));
        return new SimpleMatrix(new double[][]{{Xraw[0]},{Xraw[1]}});
    }
    public double[] regularizedTargets(double pX, double pY) {
        if(pX*pX + pY*pY > Math.hypot(proxima.length, dista.length)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (proxima.length + dista.length- 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        return new double[]{pX, pY};
    }
    public double[] inverseKinematics(double pX, double pY){
        pX -= this.x0;
        pY -= this.y0;
        //System.out.println((Math.abs(pX) < 0.01) + " "+ (-Math.signum(angleDista)));
        double signAngle = Math.abs(pX) < 0.01 ? -Math.signum(angleDista) : pX > 0 ? 1 : -1;
        if(pX*pX + pY*pY > Math.pow(proxima.length + dista.length, 2)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (proxima.length + dista.length - 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        double positionDistance = Math.hypot(pX, pY);
        //System.out.println(lenProxi + lenDista + " " + positionDistance);
 
        double angleProximal = Math.atan2(pY, pX) + signAngle * Math.acos((positionDistance * positionDistance + proxima.length * proxima.length - dista.length * dista.length)/(2 * proxima.length * positionDistance));
        double angleDistal = signAngle * (-Math.PI + Math.acos((proxima.length * proxima.length + dista.length * dista.length - positionDistance * positionDistance)/(2 * dista.length * proxima.length)));
        angleDistal = angleBound(angleDistal + angleProximal);
        if(angleDistal > Math.PI/2) angleDistal -= 2 * Math.PI;
        return new double[]{angleBound(angleProximal), angleDistal};
    }
    private SimpleMatrix inverseKinematics(SimpleMatrix Q) {
        double[] Xraw = inverseKinematics(Q.get(0), Q.get(1));
        return new SimpleMatrix(new double[][]{{Xraw[0]},{Xraw[1]}});
    }
    private SimpleMatrix jacobianEE(){
        double L1 = proxima.length;
        double L2 = dista.length;
        double t1 = angleProxima;
        double t2 = angleDista;
        double[][] cronck = {
            {-L1*sin(t1), -L2*sin(t2)},
            {L1*cos(t1), L2*cos(t2)}
        };
        SimpleMatrix jacobian = new SimpleMatrix(cronck);
        return jacobian;
    }
}