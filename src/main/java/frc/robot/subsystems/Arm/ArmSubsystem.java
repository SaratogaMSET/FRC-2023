package frc.robot.subsystems.Arm;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ejml.simple.SimpleMatrix;
import frc.robot.controls.ArmInterface;

public class ArmSubsystem extends SubsystemBase {
    public ArmInterface armInterface = new ArmInterface();
    public ArmSubsystem() {
        armInterface.resetMotorEncoders();
    }

    public void updateState(){
        armInterface.update_state();
        armInterface.showState();
    }
    public void voltageMotors(double proximal_control_voltage, double distal_control_voltage){
        armInterface.voltageMotors_SimpleFF(proximal_control_voltage, distal_control_voltage);
    }
    public double[] forwardKinematics(){
        double[] fK = armInterface.Arm.forwardKinematics(armInterface.getPositionProximal(), armInterface.getPositionDistal());
        return new double[]{fK[0], fK[1]};
    }
    public double[] inverseKinematics(double x, double y){
        double[] iK = armInterface.Arm.inverseKinematics(x, y);
        return new double[]{iK[0], iK[1]};
    }
    // public void VelocityCartesian(double vx, double vy){
    //     SimpleMatrix target_xDot = new SimpleMatrix(new double[][]{{vx}, {vy}});
    //     SimpleMatrix inverse_jacobian = Arm.jacobianEE().invert();
        
    //     SimpleMatrix target_qDot = inverse_jacobian.mult(target_xDot);

    //     double proximal_kV = 2;
    //     double distal_kV = 2;

    //     double voltageProximal = proximal_kV * target_qDot.get(0);
    //     double voltageDistal = distal_kV * target_qDot.get(1);

    //     double max_voltage = 1.5;
    //     if(Math.abs(voltageProximal) > max_voltage){
    //         voltageDistal = 0;
    //         voltageProximal = 0;
    //         System.out.println("Feedback Exceeds Proximal");
    //     }
    //     if(Math.abs(voltageDistal) > max_voltage){
    //         voltageProximal = 0;
    //         voltageDistal = 0;
    //         System.out.println("Feedback Exceeds Distal");
    //     }

    //     SmartDashboard.putNumber("CartV Prox", voltageProximal);
    //     SmartDashboard.putNumber("CartV Dist", voltageDistal);

    //     voltageMotors(voltageProximal, voltageDistal);
    // }
    public void PIDtoAngles(double target_proximal, double target_distal){
        SimpleMatrix target = new SimpleMatrix(new double[][]{{target_proximal}, {target_distal}, {0}, {0}});
        SimpleMatrix state = armInterface.state();

        SimpleMatrix error = target.minus(state);

        double proxKp = 8.2;
        double distKp = 7.5;

        double proxKd = 0.01;
        double distKd = 0.01;

        double proxKf = 0.90;
        double distKf = 1.30;
        double armTolerance = 0.03;

        double maxVoltPerVelocity = 3.0000254;
        double max_voltage = 6.5;

        double voltageProximal = error.get(0) * proxKp + error.get(2) * proxKd;
        double voltageDistal = error.get(1) * distKp + error.get(3) * distKd;

        if(Math.abs(voltageProximal) > Math.abs(maxVoltPerVelocity * state.get(2))){
            if(voltageProximal * state.get(2) < 0){
                //If Opp Direction
                voltageProximal = Math.signum(voltageProximal) * proxKf;
            }
            voltageProximal = Math.signum(error.get(0)) * (Math.abs(maxVoltPerVelocity * state.get(2)) + proxKf);
        }
        if(Math.abs(voltageDistal) > Math.abs(maxVoltPerVelocity * state.get(3))){
            if(voltageDistal * state.get(2) < 0){
                //If Opp Direction
                voltageDistal = Math.signum(voltageDistal) * distKf;
            }
            voltageDistal = Math.signum(error.get(1)) * (Math.abs(maxVoltPerVelocity * state.get(3)) + distKf);
        }

        if(Math.abs(voltageProximal) < proxKf){
            voltageProximal = Math.signum(error.get(0)) * proxKf;
        }
        if(Math.abs(voltageDistal) < distKf){
            voltageDistal = Math.signum(error.get(1)) * distKf;
        }

        if(Math.abs(error.get(0)) < armTolerance){
            voltageProximal = proxKp * error.get(0);
        }
        if(Math.abs(error.get(1)) < armTolerance){
            voltageDistal = distKp * error.get(1);
        }

        if(Math.abs(voltageProximal) > max_voltage){
            voltageProximal = Math.signum(voltageProximal) * max_voltage;
            System.out.println("Feedback Exceeds Proximal");
        }
        if(Math.abs(voltageDistal) > max_voltage){
            voltageDistal = Math.signum(voltageDistal) * max_voltage;
            System.out.println("Feedback Exceeds Distal");
        }

        SmartDashboard.putNumber("PID V Prox", voltageProximal);
        SmartDashboard.putNumber("PID V Dist", voltageDistal);

        voltageMotors(voltageProximal, voltageDistal);
    }

  @Override
    public void periodic() {
        updateState();
    }

@Override
    public void simulationPeriodic() {
// This method will be called once per scheduler run during simulation
    }
}