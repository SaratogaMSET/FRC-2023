package frc.robot.controls;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;

import org.ejml.simple.SimpleMatrix;

public class ArmKinematics {
    protected Joint J1;
	protected Joint J2;

    public void controlLoop(){

    }
	public ArmKinematics(Joint Joint1, Joint Joint2) {
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
			{J1.L * J2.L * J2.m * omega2 * omega2 * Math.sin(theta1 - theta2),
			-J1.L * J2.L * J2.m * omega1 * omega1 * Math.sin(theta1 - theta2)}
		});
	}
	public SimpleMatrix gravityMatrix(SimpleMatrix x) {
        double theta1 = x.get(0);
        double theta2 = x.get(1);
		return new SimpleMatrix(new double[][] 
		{
			{(J1.CoM * J1.m + J2.L * J2.m) * Math.cos(theta1),
			  J2.CoM * J2.m              * Math.cos(theta2)}
		}).scale(-9.806);

	}
	public SimpleMatrix appliedForce(SimpleMatrix x, SimpleMatrix qddot) {
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		return massMatrix(x).mult(qddot).plus(pdotvMatrix(x)).minus(gravityMatrix(x));
	}
	public SimpleMatrix dynamics(SimpleMatrix x, SimpleMatrix u) {
		// u = M(q)q̈ + τ_g(q)+ C(q, q̇)
		// q̈ = M⁻¹(q)(u - τ_g(q) − C(q, q̇))
		SimpleMatrix M = massMatrix(x);
		SimpleMatrix C = pdotvMatrix(x);
		SimpleMatrix g = gravityMatrix(x);
		return M.invert().mult(u.minus(C).plus(g));
	}
    public Matrix<N4, N1> dynamics(Matrix<N4, N1> state, Matrix<N2, N1> control){
        SimpleMatrix dynamics = dynamics(state.getStorage(), control.getStorage());
        return new Matrix<>(dynamics);
    }
}