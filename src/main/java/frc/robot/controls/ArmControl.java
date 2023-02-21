package frc.robot.controls;

import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.NumericalJacobian;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class ArmControl {
    protected ArmKinematics Arm;

    protected final double dT = 1.0 / 50;
    protected BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> dynamics;
    protected Vector<N4> q;
    protected Vector<N2> r;

    protected LinearSystem<N4, N2, N0> plant;
    protected LinearQuadraticRegulator<N4, N2, N0> feedback;
    protected ControlAffinePlantInversionFeedforward<N4, N2> feedforward;

    public ArmControl(){
        Joint Joint1 = new Joint(2, 2, 2, 2);
        Joint Joint2 = new Joint(2, 2, 2, 2);

        Arm = new ArmKinematics(Joint1, Joint2);

        dynamics = (x, u) -> Arm.dynamics(x, u);

        double thetaDeviation = 0.01;
        double omegaDeviation = 0.1;
        q = VecBuilder.fill(
            1/thetaDeviation * thetaDeviation,
            1/thetaDeviation * thetaDeviation,
            1/omegaDeviation * omegaDeviation,
            1/omegaDeviation * omegaDeviation);

        double proximalControlCost = 100;
        double distalControlCost = 75;
        r = VecBuilder.fill(proximalControlCost, distalControlCost);


        feedforward = new ControlAffinePlantInversionFeedforward<>(Nat.N4(), Nat.N2(), dynamics, dT);
    }
    private void linearize(SimpleMatrix state, SimpleMatrix control){
        Matrix<N4, N1> x = new Matrix<>(state);
        Matrix<N2, N1> u = new Matrix<>(control);

        Matrix<N4, N4> A = NumericalJacobian.numericalJacobianX(Nat.N4(), Nat.N4(), dynamics, x, u);
        Matrix<N4, N2> B = NumericalJacobian.numericalJacobianU(Nat.N4(), Nat.N2(), dynamics, x, u);
        plant = new LinearSystem<>(A, B, null, null);
        feedback = new LinearQuadraticRegulator<>(plant, q, r, dT);
    }
    public SimpleMatrix control(SimpleMatrix reference, SimpleMatrix state, SimpleMatrix control){
        return control(new Matrix<>(reference), new Matrix<>(state), new Matrix<>(control)).getStorage();
    }
    private Matrix<N2, N1> control(Matrix<N4, N1> reference, Matrix<N4, N1> state, Matrix<N2, N1> control){
        linearize(state.getStorage(), control.getStorage());
        Matrix<N2, N1> u_ff = feedforward.calculate(reference);
        Matrix<N2, N1> u_fb = feedback.calculate(state, reference);
        return u_ff.plus(u_fb);
    }
}
