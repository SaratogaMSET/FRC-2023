package frc.robot.controls;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;

import org.ejml.simple.SimpleMatrix;

public class ArmDynamics {
    protected Joint J1;
	protected Joint J2;

    public void controlLoop(){

    }
	public ArmDynamics(Joint Joint1, Joint Joint2) {
		this.J1 = Joint1;
		this.J2 = Joint2;
	}
	public SimpleMatrix massMatrix(SimpleMatrix x) {
        double theta1 = x.get(0);
        double theta2 = x.get(1);
		return new SimpleMatrix(new double[][] 
		{
			{J1.I + J1.L * J1.L * (J1.m + J2.m), J1.L * J2.L * J2.m * Math.cos(theta1 - theta2)},
			{J1.L * J2.L * J2.m * Math.cos(theta1 - theta2), J2.I + J2.L * J2.L * J2.m}
		});
	}
	public SimpleMatrix pdotvMatrix(SimpleMatrix x) {
        double theta1 = x.get(0);
        double theta2 = x.get(1);
        double omega1 = x.get(2);
        double omega2 = x.get(3);
		return new SimpleMatrix(new double[][] 
		{
			{J1.L * J2.L * J2.m * omega2 * omega2 * Math.sin(theta1 - theta2)},
			{-J1.L * J2.L * J2.m * omega1 * omega1 * Math.sin(theta1 - theta2)}
		});
	}
	public SimpleMatrix gravityMatrix(SimpleMatrix x) {
        double theta1 = x.get(0);
        double theta2 = x.get(1);
		return new SimpleMatrix(new double[][] 
		{	//Added Cross Joint Gravity Term
			{(J1.CoM * J1.m + J1.L * J2.m + J2.CoM * J2.m) * Math.cos(theta1)},
			{ J2.CoM * J2.m                                * Math.cos(theta2)}
		}).scale(-9.806);

	}
	public SimpleMatrix appliedForce(SimpleMatrix x, SimpleMatrix qddot) {
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		return massMatrix(x).mult(qddot).plus(pdotvMatrix(x)).minus(gravityMatrix(x));
	}
	public SimpleMatrix counteractErroneousForces(SimpleMatrix x) {
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		// q̈ = M⁻¹(q)(u - τ_g(q) − C(q, q̇))
		SimpleMatrix C = pdotvMatrix(x);
		SimpleMatrix g = gravityMatrix(x);

		return C.minus(g);
	}
	public SimpleMatrix massDynamics(SimpleMatrix x, SimpleMatrix u){
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		// q̈ = M⁻¹(q)(u - τ_g(q) − C(q, q̇))
		SimpleMatrix M = massMatrix(x);
		SimpleMatrix qDot = new SimpleMatrix(4, 1);
		SimpleMatrix qDdot = M.invert().mult(u);
		qDot.set(0, 0, x.get(2, 0));
		qDot.set(1, 0, x.get(3, 0));
		qDot.set(2, 0, qDdot.get(0, 0));
		qDot.set(3, 0, qDdot.get(1, 0));

		return qDot;
	}
	public SimpleMatrix dynamics(SimpleMatrix x, SimpleMatrix u) {
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		// q̈ = M⁻¹(q)(u - τ_g(q) − C(q, q̇))
		SimpleMatrix M = massMatrix(x);
		SimpleMatrix C = pdotvMatrix(x);
		SimpleMatrix g = gravityMatrix(x);
		SimpleMatrix qDot = new SimpleMatrix(4, 1);
		SimpleMatrix qDdot = M.invert().mult(u.minus(C).plus(g));
		qDot.set(0, 0, x.get(2, 0));
		qDot.set(1, 0, x.get(3, 0));
		qDot.set(2, 0, qDdot.get(0, 0));
		qDot.set(3, 0, qDdot.get(1, 0));

		return qDot;
	}
    public Matrix<N4, N1> dynamics(Matrix<N4, N1> state, Matrix<N2, N1> control){
        SimpleMatrix dynamics = dynamics(state.getStorage(), control.getStorage());
        return new Matrix<>(dynamics);
    }
	public Matrix<N4, N1> massDynamics(Matrix<N4, N1> state, Matrix<N2, N1> control){
        SimpleMatrix dynamics = massDynamics(state.getStorage(), control.getStorage());
        return new Matrix<>(dynamics);
    }
}