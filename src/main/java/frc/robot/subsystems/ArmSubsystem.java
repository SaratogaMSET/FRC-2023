package frc.robot.subsystems;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.controls.ArmControl;
import frc.robot.controls.ArmDynamics;
import frc.robot.controls.ArmMassControl;
import frc.robot.controls.ArmKinematics;
import frc.robot.controls.Joint;
import frc.robot.controls.VelocityFromEncoder;
import frc.robot.controls.VoltageControl;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    ArmVisualizer visualizerMeasured;
    public ArmSubsystem() {
        motorProximal.setInverted(false);
        motorDistal.setInverted(true);
        motorProximal.setIdleMode(IdleMode.kCoast);
        motorDistal.setIdleMode(IdleMode.kCoast);
        
        visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    }

    
    private final double[] boundsProxima = new double[]{Math.toRadians(35), Math.toRadians(145)};
    private final double[] boundsDistal = new double[]{Math.toRadians(-180 -20), Math.toRadians(20)};
    private final double kGP = 0.29;
    private final double kpP = 3.5;
    private final double minVP = 1.25;

    private final double kGD = 0.02;
    private final double kpD = 3.1;
    private final double minVD = 1;

    private final double pounds_to_kilograms = 0.453592;
    private final double inches_to_meters = 0.0254;
    private final double lbs_sqinches_to_kg_sqmeters = pounds_to_kilograms * inches_to_meters * inches_to_meters;

    private final Joint proximal = new Joint(17.5 * inches_to_meters, 1.6 * pounds_to_kilograms, 236.59 * lbs_sqinches_to_kg_sqmeters, 9.88 * inches_to_meters);
    private final Joint distal = new Joint(14.5 * inches_to_meters, 0.88 * pounds_to_kilograms, 25.01 * lbs_sqinches_to_kg_sqmeters, 2.15 * inches_to_meters);

    public ArmMassControl ArmControl = new ArmMassControl(proximal, distal);
    public ArmKinematics Arm = new ArmKinematics(proximal, distal);
    public CANSparkMax motorProximal = new CANSparkMax(Constants.Arm.proximalMotorID, MotorType.kBrushless);
    public CANSparkMax motorDistal = new CANSparkMax(Constants.Arm.distalMotorID, MotorType.kBrushless);

    DutyCycleEncoder encProximal = new DutyCycleEncoder(9);
    DutyCycleEncoder encDistal = new DutyCycleEncoder(8);
    VelocityFromEncoder velocityProximal = new VelocityFromEncoder(getProximalRadians(), 7);
    VelocityFromEncoder velocityDistal = new VelocityFromEncoder(getProximalRadians(), 7);

    double convertRotRad = 2 * Math.PI;

    public void updateState(){
        velocityProximal.update(getProximalRadians());
        velocityDistal.update(getDistalRadians());
        Arm.setAngles(getProximalRadians(), getDistalRadians());
        Arm.setOmegas(getProximalRadiansPerSecond(), getDistalRadiansPerSecond());
    }
    public double[] forwardKinematics(){
        double[] fK = Arm.forwardKinematics(getProximalRadians(), getDistalRadians());
        return new double[]{fK[0], fK[1]};
    }
    public double[] inverseKinematics(double x, double y){
        double[] iK = Arm.inverseKinematics(x, y);
        return new double[]{iK[0], iK[1]};
    }
    public void LQRtoAngles(double target_proximal, double target_distal){
        SimpleMatrix feedback = ArmControl.control(new SimpleMatrix(new double[][]{{target_proximal}, {target_distal}, {0}, {0}}), new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {0}, {0}}));
        double voltageProximal = VoltageControl.GearedNEOVoltage(feedback.get(0), getProximalRadiansPerSecond(), 0, Constants.Arm.gearProximalReduction);
        double voltageDistal = VoltageControl.GearedNEOVoltage(feedback.get(1), getDistalRadiansPerSecond(), 0, Constants.Arm.gearDistalReduction);
        //voltageMotors(voltageProximal, voltageDistal);
    }
    public void voltageMotors(double controlVoltageProxima, double controlVoltageDistal){
        if(getProximalRadians() < boundsProxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getProximalRadians() > boundsProxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getDistalRadians() < boundsDistal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getDistalRadians() > boundsDistal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;

        SimpleMatrix counteractingTorquesErroneous = ArmControl.counteractErroneousForces(new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {getProximalRadiansPerSecond()}, {getDistalRadiansPerSecond()}}));

        double tq_ff_prox = counteractingTorquesErroneous.get(0);
        double tq_ff_dist = counteractingTorquesErroneous.get(1);
        double voltage_ff_prox = VoltageControl.GearedNEOVoltage(tq_ff_prox, 0, 0, Constants.Arm.gearProximalReduction);
        double voltage_ff_dist = VoltageControl.GearedNEOVoltage(tq_ff_dist, 0, 0, Constants.Arm.gearDistalReduction);

        motorProximal.setVoltage(controlVoltageProxima + voltage_ff_prox);
        motorDistal.setVoltage(controlVoltageDistal + voltage_ff_dist);
    }
    public void rawVoltageMotors(double controlVoltageProxima, double controlVoltageDistal){
        if(getProximalRadians() < boundsProxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getProximalRadians() > boundsProxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getDistalRadians() < boundsDistal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getDistalRadians() > boundsDistal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;
        motorProximal.setVoltage(controlVoltageProxima);
        motorDistal.setVoltage(controlVoltageDistal);
    }
    
    public double getProximalRadians(){
        double angle = -(encProximal.getAbsolutePosition() - 0.7664) * (2 * Math.PI) + Math.PI/2;
        if(angle < 0) angle += 2*Math.PI;
        if(angle > 2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }
    public double getDistalRadians(){
        double angle = -(encDistal.getAbsolutePosition() - 0.07611) * (2 * Math.PI) - Math.PI + getProximalRadians();
        if(angle < -3.0/2 * Math.PI) angle += 2*Math.PI;
        if(angle > Math.PI/2) angle -= 2*Math.PI;
        return angle;
    }
    public double getProximalRadiansPerSecond(){
        return velocityProximal.getRawVelocity();
    }
    public double getDistalRadiansPerSecond(){
        return velocityDistal.getRawVelocity();
    }
  @Override
    public void periodic() {
        updateState();

        //ArmControl.getKMatrix(new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {getProximalRadiansPerSecond()}, {getDistalRadiansPerSecond()}})).print();

        SmartDashboard.putNumber("Raw ProxEnc", encProximal.getAbsolutePosition());
        SmartDashboard.putNumber("Raw DistEnc", encDistal.getAbsolutePosition());
        boolean ShowBounds = false;
        if(ShowBounds){
            SmartDashboard.putBoolean("lb Prox", getProximalRadians() < boundsProxima[0]);
            SmartDashboard.putBoolean("hb Prox", getProximalRadians() > boundsProxima[1]);
            SmartDashboard.putBoolean("lb Dist", getDistalRadians() < boundsDistal[0]);
            SmartDashboard.putBoolean("hb Dist", getDistalRadians() > boundsDistal[1]);
        }
        boolean ShowCurrent = false;
        if(ShowCurrent){
            SmartDashboard.putNumber("Proxima current", motorProximal.getOutputCurrent());
            SmartDashboard.putNumber("Distal current", motorDistal.getOutputCurrent());
        }
        
        boolean ShowPositionals = true;
        if(ShowPositionals){
            SmartDashboard.putNumber("Proxima Deg", 180 / Math.PI * getProximalRadians());
            SmartDashboard.putNumber("Distal Deg", 180 / Math.PI * getDistalRadians());
            SmartDashboard.putNumber("Proxima Deg/s", 180 / Math.PI * getProximalRadiansPerSecond());
            SmartDashboard.putNumber("Distal Deg/s", 180 / Math.PI * getDistalRadiansPerSecond());

            double[] fK = forwardKinematics();
            SmartDashboard.putNumber("pX", fK[0]);
            SmartDashboard.putNumber("pY", fK[1]);
            double[] iK = Arm.inverseKinematics(fK[0], fK[1]);
            SmartDashboard.putNumber("q1", 180 / Math.PI * iK[0]);
            SmartDashboard.putNumber("q2", 180 / Math.PI * iK[1]);
            double[] xdot = Arm.cartesianSpeed();
            SmartDashboard.putNumber("vX", xdot[0]);
            SmartDashboard.putNumber("vY", xdot[1]);
        }
        boolean ShowRawTorques = true;
        if (ShowRawTorques){ 
            // SimpleMatrix testControl = ArmControl.control(new SimpleMatrix(new double[][]{{3.1415/2}, {-3.1415/2}, {0}, {0}}), new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {0}, {0}}));
            // SmartDashboard.putNumber("ArmControl Proximal", testControl.get(0));
            // SmartDashboard.putNumber("ArmControl Distal", testControl.get(1));
            SimpleMatrix currentState = new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {getProximalRadiansPerSecond()}, {getDistalRadiansPerSecond()}});
            //SimpleMatrix testTorquesErroneous = ArmControl.counteractErroneousForces(currentState);
            SimpleMatrix GravityTorques = ArmControl.Arm.gravityMatrix(currentState);
            SimpleMatrix CentrifugalTorques = ArmControl.Arm.pdotvMatrix(currentState);


            SmartDashboard.putNumber("TqProx Grav", GravityTorques.get(0));
            SmartDashboard.putNumber("TqDist Grav", GravityTorques.get(1));
            SmartDashboard.putNumber("TqProx PdotV", CentrifugalTorques.get(0));
            SmartDashboard.putNumber("TqDist PdotV", CentrifugalTorques.get(1));
        }
        boolean ShowRawVoltages = true;
        if (ShowRawVoltages){
            //SimpleMatrix testControl = ArmControl.control(new SimpleMatrix(new double[][]{{3.1415/2}, {-3.1415/2}, {0}, {0}}), new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {0}, {0}}));
            SimpleMatrix testTorquesErroneous = ArmControl.counteractErroneousForces(new SimpleMatrix(new double[][]{{getProximalRadians()}, {getDistalRadians()}, {getProximalRadiansPerSecond()}, {getDistalRadiansPerSecond()}}));
            //double tq_fb_prox = testControl.get(0);
            //double tq_fb_dist = testControl.get(1);

            double tq_ff_prox = testTorquesErroneous.get(0);
            double tq_ff_dist = testTorquesErroneous.get(1);
            //SmartDashboard.putNumber("ArmControlV Proximal", VoltageControl.GearedNEOVoltage(tq_fb_prox, 0, 0, Constants.Arm.gearProximalReduction));
            //SmartDashboard.putNumber("ArmControlV Distal", VoltageControl.GearedNEOVoltage(tq_fb_dist, 0, 0, Constants.Arm.gearDistalReduction));

            
            SmartDashboard.putNumber("Volts Proximal Erroneous", VoltageControl.GearedNEOVoltage(tq_ff_prox, 0, 0, Constants.Arm.gearProximalReduction));
            SmartDashboard.putNumber("Volts Distal Erroneous", VoltageControl.GearedNEOVoltage(tq_ff_dist, 0, 0, Constants.Arm.gearDistalReduction));
        }


        visualizerMeasured.update(getProximalRadians(), getDistalRadians());
    }

@Override
    public void simulationPeriodic() {
// This method will be called once per scheduler run during simulation
    }
}