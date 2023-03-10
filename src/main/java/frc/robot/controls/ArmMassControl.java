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

public class ArmMassControl {
    public ArmDynamics Arm;

    protected final double dT = 1.0 / 50;
    protected BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> dynamics;
    protected Vector<N4> q;
    protected Vector<N2> r;

    protected LinearSystem<N4, N2, N0> plant;
    protected LinearQuadraticRegulator<N4, N2, N0> feedback;

    Matrix<N2, N4> previousK;

    public ArmMassControl(Joint proxima, Joint distal){
        Arm = new ArmDynamics(proxima, distal);

        dynamics = (x, u) -> Arm.massDynamics(x, u);
        double thetaDeviation = 0.0005;
        double omegaDeviation = 0.0005;
        q = VecBuilder.fill(
            thetaDeviation,
            thetaDeviation,
            omegaDeviation,
            omegaDeviation);

        double proximalControlCost = 25;
        double distalControlCost = 16;
        r = VecBuilder.fill(proximalControlCost, distalControlCost);
    }
    private void linearize(SimpleMatrix state, SimpleMatrix control){
        Matrix<N4, N1> x = new Matrix<>(state);
        Matrix<N2, N1> u = new Matrix<>(control);

        Matrix<N4, N4> A = NumericalJacobian.numericalJacobianX(Nat.N4(), Nat.N4(), dynamics, x, u);
        Matrix<N4, N2> B = NumericalJacobian.numericalJacobianU(Nat.N4(), Nat.N2(), dynamics, x, u);

        Matrix<N0, N4> C = new Matrix<>(new SimpleMatrix(0, 4));
        Matrix<N0, N2> D = new Matrix<>(new SimpleMatrix(0, 2));

        plant = new LinearSystem<>(A, B, C, D);
        feedback = new LinearQuadraticRegulator<>(plant, q, r, dT);
    }
    public SimpleMatrix counteractErroneousForces(SimpleMatrix state){
        return Arm.counteractErroneousForces(state);
    }
    public SimpleMatrix control(SimpleMatrix reference, SimpleMatrix state){
        SimpleMatrix control = new SimpleMatrix(2, 1);
        return control(new Matrix<>(reference), new Matrix<>(state), new Matrix<>(control)).getStorage();
    }
    public SimpleMatrix control(SimpleMatrix reference, SimpleMatrix state, SimpleMatrix control){
        return control(new Matrix<>(reference), new Matrix<>(state), new Matrix<>(control)).getStorage();
    }
    private Matrix<N2, N1> control(Matrix<N4, N1> reference, Matrix<N4, N1> state, Matrix<N2, N1> control){
        linearize(state.getStorage(), control.getStorage());
        Matrix<N2, N4> K;

        try{
            K = feedback.getK();
            previousK = K;
        }catch(Exception e){
            System.out.println("Arm Linearization Failed");
            K = previousK;
        }
        
        Matrix<N2, N1> u_fb = K.times(reference.minus(state));
        return u_fb;
    }
}