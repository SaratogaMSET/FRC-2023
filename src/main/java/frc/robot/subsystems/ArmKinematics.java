package frc.robot.subsystems;

import org.ejml.simple.*;

import edu.wpi.first.math.util.Units;
 
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
 
    private double angleProxima;
    private double angleDista;
    private double omegaProxima;
    private double omegaDista;

    Joint proxima = new Joint(Units.inchesToMeters(17.5), 0, 0.573 + 0.210);
    Joint dista = new Joint(Units.inchesToMeters(14.5), 0, 0.488 + 0.210);
 
    public class Joint {
        public Joint(double displacementX, double displacementY, double mass){
            this.tX = displacementX;
            this.tY = displacementY;
            this.m = mass;
        }
        double tX;
        double tY;
        double m;
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
    private SimpleMatrix jointMass(Joint J){
        double[][] kronck = {
            {J.m, 0},
            {0, J.m}
        };
        SimpleMatrix jMassMat = new SimpleMatrix(kronck);
        return jMassMat;
    }
    public double[] forwardKinematics(double angProxi, double angleDistal){
    	double L1 = proxima.tX;
        double L2 = dista.tX;
        double O1 = proxima.tY;
        double O2 = dista.tY;
        double t1 = angleProxima;
        double t2 = angleDista;
        return new double[]{
        	L1*cos(t1) - O1*sin(t1) + x0 - (L2*sin(t2) + O2*cos(t2))*sin(t1) + (L2*cos(t2) - O2*sin(t2))*cos(t1),
        	L1*sin(t1) + O1*cos(t1) + y0 + (L2*sin(t2) + O2*cos(t2))*cos(t1) + (L2*cos(t2) - O2*sin(t2))*sin(t1)
        }; 
    }
    public double[] forwardKinematicsJ1(double angProxi) {
    	double L1 = proxima.tX;
        double O1 = proxima.tY;
        double t1 = angleProxima;
        return new double[]{L1*cos(t1) - O1*sin(t1) + x0, L1*sin(t1) + O1*cos(t1) + y0};
    }
    private SimpleMatrix forwardKinematics(SimpleMatrix Q) {
    	double[] Xraw = forwardKinematics(Q.get(0), Q.get(1));
    	return new SimpleMatrix(new double[][]{{Xraw[0]},{Xraw[1]}});
    }
    public double[] inverseKinematics(double pX, double pY){
        boolean relativePositiveAngle;
        if(pX < 0) relativePositiveAngle = true;
        else relativePositiveAngle = false;
 
        double lenProxi = Math.hypot(proxima.tY, proxima.tX);
        double angleOffsetProxi = Math.atan2(proxima.tY, proxima.tX);
        double lenDist = Math.hypot(dista.tY, dista.tX);
        double angleOffsetDista = Math.atan2(dista.tY, dista.tX);
        double signAngle = 1;
        double positionDistance = Math.hypot(pX, pY);
        if(!relativePositiveAngle) signAngle = -1;
 
        double angleProximal = Math.atan2(pY, pX) + signAngle * Math.acos((positionDistance * positionDistance + lenProxi * lenProxi - lenDist * lenDist)/(2 * lenProxi * positionDistance)) - angleOffsetProxi;
        double angleDistal = signAngle * (Math.acos((lenProxi * lenProxi + lenDist * lenDist - positionDistance * positionDistance)/(2 * lenDist * lenProxi))) + angleOffsetProxi - angleOffsetDista;
        return new double[]{angleProximal, angleDistal};
      }
    private SimpleMatrix inverseKinematics(SimpleMatrix Q) {
    	double[] Xraw = inverseKinematics(Q.get(0), Q.get(1));
    	return new SimpleMatrix(new double[][]{{Xraw[0]},{Xraw[1]}});
    }
    //NOT FOR TARGET VELOCITY, for current velocity
    public double[] jointVelocity(double omegaProxima, double omegaDista){
        // double[][] xVelDesRaw = {{velX}, {velY}};
    	// SimpleMatrix xVelDes = new SimpleMatrix(xVelDesRaw);
        // SimpleMatrix signal = jacobianJ2().invert().mult(xVelDes);
 
        double[] toReturn = new double[2];
        toReturn[0] = omegaProxima;
        toReturn[1] = omegaDista;
 
        if(angleProxima < boundsProxima[0] && Math.signum(toReturn[0]) == -1) toReturn[0] = 0;
        if(angleProxima > boundsProxima[1] && Math.signum(toReturn[0]) == 1) toReturn[0] = 0;
        if(angleDista < boundsDista[0] && Math.signum(toReturn[1]) == -1) toReturn[0] = 0;
        if(angleDista > boundsDista[1] && Math.signum(toReturn[1]) == 1) toReturn[0] = 0;
        return toReturn;
    }
    public double[] torquePID(double px, double py, double vx, double vy){
        final double kP = 32;
        final double kV = 9;
        final double kPuNull = 0;
 
        SimpleMatrix Q = new SimpleMatrix(new double[][] {{angleProxima}, {angleDista}});
        SimpleMatrix Qdot = new SimpleMatrix(new double[][] {{omegaProxima}, {omegaDista}});
 
        SimpleMatrix refpX = new SimpleMatrix(new double[][] {{px}, {py}});
        SimpleMatrix refpQ = inverseKinematics(refpX);
        //forwardKinematics(refpQ).print();
        SimpleMatrix refvX = new SimpleMatrix(new double[][] {{vx}, {vy}});
 
        SimpleMatrix curpX = forwardKinematics(Q); //Current Position
        SimpleMatrix curvX = jacobianJ2().mult(Qdot);
 
        SimpleMatrix desiredAcc = refpX.minus(curpX).scale(kP).plus((refvX.minus(curvX)).scale(kV)); // = kV*(vDes - v) + kP*(pDes - p);
        SimpleMatrix uNull = refpQ.minus(Q).scale(kPuNull); //Need to add sigmoid scaling term based on how far it is from nominal
 
        SimpleMatrix torques = inertiaController(desiredAcc).plus(nullSpaceFilter(uNull));
        
        //Account for bounds
        if(angleProxima < boundsProxima[0] && Math.signum(torques.get(0)) == -1) torques.set(0, 0);
        if(angleProxima > boundsProxima[1] && Math.signum(torques.get(0)) == 1) torques.set(0, 0);
        if(angleDista < boundsDista[0] && Math.signum(torques.get(1)) == -1) torques.set(1, 0);
        if(angleDista > boundsDista[1] && Math.signum(torques.get(1)) == 1) torques.set(1, 0);

        
        torques = torques.plus(gravityQ());
        return new double[] {torques.get(0), torques.get(1)};
    }
    public double[] jointTorque(double accelX, double accelY){
        SimpleMatrix gravityTorque = gravityQ();
    	double[][] xAccDesRaw = {{accelX}, {accelY}};
    	SimpleMatrix xAccDes = new SimpleMatrix(xAccDesRaw);
        SimpleMatrix signal = inertiaController(xAccDes).plus(gravityQ());
 
        double[] toReturn = new double[2];
        toReturn[0] = signal.get(0);
        toReturn[1] = signal.get(1);
 
        if(angleProxima < boundsProxima[0] && Math.signum(toReturn[0]) == -1) toReturn[0] = gravityTorque.get(0);
        if(angleProxima > boundsProxima[1] && Math.signum(toReturn[0]) == 1) toReturn[0] = gravityTorque.get(0);
        if(angleDista < boundsDista[0] && Math.signum(toReturn[1]) == -1) toReturn[0] = gravityTorque.get(1);
        if(angleDista > boundsDista[1] && Math.signum(toReturn[1]) == 1) toReturn[0] = gravityTorque.get(1);
 
        return toReturn;
    }
    private SimpleMatrix inertiaController(SimpleMatrix xAccDes){
        return jacobianJ2().transpose().mult(inertiaX()).mult(xAccDes);
    }
    //private SimpleMatrix nullSpaceJacobianPseudoInverse() {
    //	return inertiaX().mult(jacobianJ2()).mult(inertiaQ().invert());
    //}
    private SimpleMatrix nullSpaceFilter(SimpleMatrix uNull){
        //return SimpleMatrix.identity(2).minus(jacobianJ2().transpose().mult(nullSpaceJacobianPseudoInverse())).mult(uNull);
        return (uNull);
    }
    private SimpleMatrix inertiaX(){
        SimpleMatrix Mx_inv = (jacobianJ2().mult(inertiaQ().invert()).mult(jacobianJ2().transpose()));
        if(Math.abs(jacobianJ2().mult(jacobianJ2().transpose()).determinant()) > Math.pow(0.005, 2)){
        	return Mx_inv.invert();
        }else {
        	//Regularization
        	SimpleSVD<SimpleMatrix> MXSVD = Mx_inv.svd();
        	for(int i = 0; i < MXSVD.getW().numCols(); i++) {
        		System.out.println("Regularization Triggered");
        		if(MXSVD.getW().get(i, i) < 0.005) MXSVD.getW().set(i, i, 0);
        		else {
        			MXSVD.getW().set(i, i, 1/MXSVD.getW().get(i, i));
        		}
        	}
        	return MXSVD.getU().mult(MXSVD.getW()).mult(MXSVD.getU().transpose());
        }
    }
    public SimpleMatrix inertiaQ(){
        SimpleMatrix inertia1 = jacobianJ1().transpose().mult(jointMass(proxima)).mult(jacobianJ1());
        SimpleMatrix inertia2 = jacobianJ2().transpose().mult(jointMass(dista)).mult(jacobianJ2());
 
        return inertia1.plus(inertia2);
    }
    public SimpleMatrix gravityQ(){
        double[][] rawFG1 = {{0}, {-9.807 * proxima.m}};
        double[][] rawFG2 = {{0}, {-9.807 * dista.m}};
 
        SimpleMatrix FG1 = new SimpleMatrix(rawFG1);
        SimpleMatrix FG2 = new SimpleMatrix(rawFG2);
 
        SimpleMatrix gravity1 = jacobianJ1().transpose().mult(FG1);
        SimpleMatrix gravity2 = jacobianJ2().transpose().mult(FG2);
        //System.out.println("Gravity: " + gravity1.plus(gravity2));
        return gravity1.plus(gravity2);
    }
    private SimpleMatrix jacobianJ2(){
        double L1 = proxima.tX;
        double L2 = dista.tX;
        double O1 = proxima.tY;
        double O2 = dista.tY;
        double t1 = angleProxima;
        double t2 = angleDista;
        double[][] cronck = {
            {-L1*sin(t1) - O1*cos(t1) - (L2*sin(t2) + O2*cos(t2))*cos(t1) - (L2*cos(t2) - O2*sin(t2))*sin(t1), (-L2*sin(t2) - O2*cos(t2))*cos(t1) - (L2*cos(t2) - O2*sin(t2))*sin(t1)},
            {L1*cos(t1) - O1*sin(t1) - (L2*sin(t2) + O2*cos(t2))*sin(t1) + (L2*cos(t2) - O2*sin(t2))*cos(t1), (-L2*sin(t2) - O2*cos(t2))*sin(t1) + (L2*cos(t2) - O2*sin(t2))*cos(t1)}
        };
        SimpleMatrix jacobian = new SimpleMatrix(cronck);
 
 
        return jacobian;
    }
 
    private SimpleMatrix jacobianJ1(){
        double L1 = proxima.tX;
        //double L2 = dista.tX;
        double O1 = proxima.tY;
        //double O2 = dista.tY;
        double t1 = angleProxima;
        //double t2 = angleDista;
        double[][] kronc = {
            {-L1*sin(t1) - O1*cos(t1), 0},
            {L1*cos(t1) - O1*sin(t1), 0}
        };
        SimpleMatrix jacobian = new SimpleMatrix(kronc);
        return jacobian;
    }
 
}