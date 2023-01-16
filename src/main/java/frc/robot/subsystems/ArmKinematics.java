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
    Joint proxima = new Joint(0.4445, 0, 0.573 + 0.210);
    Joint dista = new Joint(0.3683, 0, 0.488 + 0.210);
 
    public class Joint {
        public Joint(double displacementX, double displacementY, double mass){
            this.tX = displacementX;
            this.tY = displacementY;
            this.m = mass;
        }
        double tX;
        double tY;
        double m;
        public double length() {
            return Math.hypot(tX, tY);
        }
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
    public double[] regularizedTargets(double pX, double pY) {
        double lenProxi = Math.hypot(proxima.tY, proxima.tX);
        double angleOffsetProxi = Math.atan2(proxima.tY, proxima.tX);
        double lenDista = Math.hypot(dista.tY, dista.tX);
        double angleOffsetDista = Math.atan2(dista.tY, dista.tX);
        double signAngle = 1;
        if(pX*pX + pY*pY > Math.pow(lenProxi + lenDista, 2)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (lenProxi + lenDista - 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        return new double[]{pX, pY};
    }
    public double[] inverseKinematics(double pX, double pY){
        pX -= this.x0;
        pY -= this.y0;
 
        double lenProxi = Math.hypot(proxima.tY, proxima.tX);
        double angleOffsetProxi = Math.atan2(proxima.tY, proxima.tX);
        double lenDista = Math.hypot(dista.tY, dista.tX);
        double angleOffsetDista = Math.atan2(dista.tY, dista.tX);
        //System.out.println((Math.abs(pX) < 0.01) + " "+ (-Math.signum(angleDista)));
        double signAngle = Math.abs(pX) < 0.01 ? -Math.signum(angleDista) : pX > 0 ? 1 : -1;
        if(pX*pX + pY*pY > Math.pow(lenProxi + lenDista, 2)) {
            //System.out.println("Position out of bounds, regularizing vector");
            double scaleFactor = (lenProxi + lenDista - 0.001)/Math.hypot(pX, pY);
            pX *= scaleFactor;
            pY *= scaleFactor;
        }
        double positionDistance = Math.hypot(pX, pY);
        //System.out.println(lenProxi + lenDista + " " + positionDistance);
 
        double angleProximal = Math.atan2(pY, pX) + signAngle * Math.acos((positionDistance * positionDistance + lenProxi * lenProxi - lenDista * lenDista)/(2 * lenProxi * positionDistance)) - angleOffsetProxi;
        double angleDistal = signAngle * (-Math.PI + Math.acos((lenProxi * lenProxi + lenDista * lenDista - positionDistance * positionDistance)/(2 * lenDista * lenProxi))) + angleOffsetProxi - angleOffsetDista;
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
    private double sigmoid(double input, double strength, double offset){
        return 1/(1+
            Math.exp(-strength * (input - offset))
        );
    }
    private double boundViolationScore(double position, double velocity, double[] bounds){
        //RETURNS A VALUE FROM 0 (CLOSE) TO 1 (FAR) BASED ON HOW CLOSE IT IS TO A BOUND
        double strength = 10;
        //double doubleSigmoid = sigmoid(input, strength, bounds[0]) * sigmoid(input, -strength, bounds[0]);
        double lowBoundSigmoid = sigmoid(position, strength, bounds[0]);
        double highBoundSigmoid = sigmoid(position, -strength, bounds[1]);
        //If it's going into the low bound
        if(Math.signum(velocity) == -1){
            return lowBoundSigmoid;
        //If it's going into the high bound
        }else if(Math.signum(velocity) == 1){
            return highBoundSigmoid;
        }else{
            //Else no violation, not accelerating
            return 0;
        }
    }
    private SimpleMatrix handleBoundTorque(SimpleMatrix rawTorques){
        double scalingProxima = Math.max(0, 2*boundViolationScore(angleProxima, omegaProxima, boundsProxima)-1);
        double scalingDistal = Math.max(0, 2*boundViolationScore(angleDista, omegaDista, boundsDista)-1);
        return new SimpleMatrix(new double[][] {{scalingProxima * rawTorques.get(0)}, {scalingDistal * rawTorques.get(1)}});
    }
    public double[] torquePID(double px, double py, double vx, double vy){
        final double kP = 15;
        final double ksP = 15;
        final double kV = 12;
        final double kDistaSlow = 0.5;
        final double distaSlowThreshold = 1;
        final double kPuNull = 2;
 
        SimpleMatrix Q = new SimpleMatrix(new double[][] {{angleProxima}, {angleDista}});
        //inverseKinematics(forwardKinematics(Q)); //To make sure no angles are +- 2pi
        SimpleMatrix Qdot = new SimpleMatrix(new double[][] {{omegaProxima}, {omegaDista}});
 
        SimpleMatrix refpX = new SimpleMatrix(new double[][] {{px}, {py}});
        SimpleMatrix refpQ = inverseKinematics(refpX);
        //inverseKinematics(forwardKinematics(refpQ)).print();
        SimpleMatrix refvX = new SimpleMatrix(new double[][] {{vx}, {vy}});
 
        SimpleMatrix curpX = forwardKinematics(Q); //Current Position
        SimpleMatrix curvX = jacobianJ2().mult(Qdot);
        
        SimpleMatrix xErr = refpX.minus(curpX);
        double xErrMag = Math.hypot(xErr.get(0), xErr.get(1));
        SimpleMatrix xErrUnit = new SimpleMatrix(2, 1);
        if(Math.hypot(xErr.get(0), xErr.get(1)) > 0.01) {
            //Becomes Bigger as it gets closer
            xErrUnit = xErr.divide(1 + xErr.get(0)*xErr.get(0) + xErr.get(1)*xErr.get(1));
            //xErrUnit.print();
        }
        SimpleMatrix xVErr = refvX.minus(curvX);
        double floorBoundError = 0.5 / (1+Math.exp(100 * curpX.get(1)));
        xVErr.set(1, xVErr.get(1) + floorBoundError);
        
        SimpleMatrix desiredAcc = xErr.scale(kP).plus(xErrUnit.scale(ksP)).plus(xVErr.scale(kV)); // = kV*(vDes - v) + kP*(pDes - p);
        SimpleMatrix torques = inertiaController(desiredAcc);
        //SimpleMatrix uNull = refpQ.minus(Q).scale(kPuNull); //Need to add sigmoid scaling term based on how far it is from nominal
        //Reasoning behind sigmoid correction: <90 deg of error arm will converge to the correct position, so apply full strength null controller if error is larger than 90deg.
        //Sigmoid ensures smooth accelerations
        double distalQError = refpQ.get(1) - Q.get(1);
        System.out.println("RefQ: " + refpQ.get(1) + " | Q: " + Q.get(1));
        //System.out.println((refpQ.get(1) + Q.get(1) + Math.PI) + ", " + sigmoidNullSpace(refpQ.get(1) + Q.get(1) + Math.PI, sigmoidSteepness));
        SimpleMatrix uNullSigmoid = new SimpleMatrix(new double[][]{{0}, {(Math.signum(distalQError) + Math.cbrt(Math.signum(distalQError) + distalQError)) * kPuNull}}); //Apply no corrections to proxima
        if(Math.signum(Q.get(1)) != Math.signum(refpQ.get(1))){
            //System.out.println("Adding Swingover, Ti: " + torques.get(1) + ", SwO c: " + uNullSigmoid.get(1));
            torques = torques.scale(1 /(1.2 + kPuNull)).plus(uNullSigmoid);
        }else {
            if(Math.abs(omegaDista) > distaSlowThreshold) torques.set(1, torques.get(1)-kDistaSlow * omegaDista);
        }
        //torques.print();
        //torques = handleBoundTorque(torques);
        
        //Account for bounds
        // if(angleProxima < boundsProxima[0] && Math.signum(torques.get(0)) == -1) torques.set(0, 0);
        // if(angleProxima > boundsProxima[1] && Math.signum(torques.get(0)) == 1) torques.set(0, 0);
        // if(angleDista < boundsDista[0] && Math.signum(torques.get(1)) == -1) torques.set(1, 0);
        // if(angleDista > boundsDista[1] && Math.signum(torques.get(1)) == 1) torques.set(1, 0);
        
        torques = torques.plus(gravityQ());
        torques.print();
        return new double[] {torques.get(0), torques.get(1)};
    }
    // public double[] torquePath(double px, double py, double desiredTime, int iter){
    //     final double kPuNull = 5;
    //     final double splineVelScaling = 0;
 
    //     SimpleMatrix Q = new SimpleMatrix(new double[][] {{angleProxima}, {angleDista}});
    //     SimpleMatrix Qdot = new SimpleMatrix(new double[][] {{omegaProxima}, {omegaDista}});
 
    //     SimpleMatrix refpX = new SimpleMatrix(new double[][] {{px}, {py}});
    //     SimpleMatrix refpQ = inverseKinematics(refpX);
 
    //     SimpleMatrix curpX = forwardKinematics(Q);
    //     SimpleMatrix curvX = jacobianJ2().mult(Qdot);
        
    //     double currentVelMag = Math.hypot(curvX.get(0), curvX.get(1));
    //     ArmPath path = new ArmPath(px, py, curpX.get(0), curpX.get(1), splineVelScaling * curvX.get(0), splineVelScaling * curvX.get(1));
    //     path.profile(currentVelMag, desiredTime);
        
    //     if(angleProxima == Math.PI/4) {
    //         //new SimpleMatrix(path.nextAccel()).print();
    //     }
        
    //     if(iter == 50 || iter == 0 || iter == 100) {
    //         curpX.print();
    //         Q.print();
    //         path.print();
    //     }
        
    //     SimpleMatrix desiredAcc = new SimpleMatrix(new double[][] {{-1}, {0}}); //new SimpleMatrix(path.nextAccel());
    //     SimpleMatrix torques = inertiaController(desiredAcc);
    //     //SimpleMatrix uNull = refpQ.minus(Q).scale(kPuNull);
    //     double distalQError = refpQ.get(1) - Q.get(1);
    //     System.out.println("RefQ: " + refpQ.get(1) + " | Q: " + Q.get(1));
    //     SimpleMatrix QCorrection = new SimpleMatrix(new double[][]{{0}, {Math.signum(distalQError) * kPuNull}}); //Apply no corrections to proxima
    //     if(Math.signum(Q.get(1)) != Math.signum(refpQ.get(1))){
    //         torques = QCorrection;
    //     }  
    //     //torques.print();
    //     //torques = handleBoundTorque(torques);
        
    //     torques = torques.plus(gravityQ());
    //     return new double[] {torques.get(0), torques.get(1)};
    // }
    private SimpleMatrix inertiaController(SimpleMatrix xAccDes){
        return jacobianJ2().transpose().mult(inertiaX()).mult(xAccDes);
    }
    //private SimpleMatrix nullSpaceJacobianPseudoInverse() {
    //  return inertiaX().mult(jacobianJ2()).mult(inertiaQ().invert());
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