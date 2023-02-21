package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        motorProximal.setInverted(true);
        motorDistal.setInverted(false);
        motorProximal.setIdleMode(IdleMode.kCoast);
        motorDistal.setIdleMode(IdleMode.kCoast);

        ext_encProximal.setDistancePerRotation(convertRotRad);
        ext_encDistal.setDistancePerRotation(convertRotRad);
    }

    private final double[] boundsProxima = new double[]{Math.toRadians(35), Math.toRadians(145)};
    private final double[] boundsDistal = new double[]{Math.toRadians(-180 -50), Math.toRadians(50)};
    private final double kGP = 0.185;
    private final double kpP = 3.5;
    private final double minVP = 1.25;

    private final double kpD = 3.1;
    private final double minVD = 1;
    public ArmKinematics Arm = new ArmKinematics(0, 0);
    public CANSparkMax motorProximal = new CANSparkMax(Constants.Arm.proximalMotorID, MotorType.kBrushless);
    public CANSparkMax motorDistal = new CANSparkMax(Constants.Arm.distalMotorID, MotorType.kBrushless);
    RelativeEncoder encoderProximal = motorProximal.getEncoder();
    RelativeEncoder encoderDistal = motorDistal.getEncoder();

    DutyCycleEncoder ext_encProximal = new DutyCycleEncoder(9);
    DutyCycleEncoder ext_encDistal = new DutyCycleEncoder(8);

    double convertRotRad = 2 * Math.PI;

    public void updateState(){
        Arm.setAngles(encoderProximal.getPosition(), encoderDistal.getPosition());
        Arm.setOmegas(encoderProximal.getVelocity(), encoderDistal.getVelocity());
    }
    public double[] forwardKinematics(){
        double[] fK = Arm.forwardKinematics(encoderProximal.getPosition(), encoderDistal.getPosition());
        return new double[]{fK[0], fK[1]};
    }
    public double gravityVoltageProxima(){
        return Math.cos(encoderProximal.getPosition()) * kGP;
    }
    public double gravityVoltageDistal(){
        return 0;
    }
    public void pidToAngles(double tAngleP, double tAngleD){
        double errorP = tAngleP - encoderProximal.getPosition();
        double voltageProximal = Math.abs(errorP * kpP) < minVP ? Math.signum(errorP) * minVP : errorP * kpP;
        
        double errorD = tAngleD - encoderDistal.getPosition();
        double voltageDistal = Math.abs(errorD * kpD) < minVD ? Math.signum(errorD) * minVD : errorD * kpD;
        
        if(Math.abs(errorP) < 0.03) voltageProximal = 0;
        if(Math.abs(errorD) < 0.03) voltageDistal = 0;
        voltageMotors(voltageProximal, voltageDistal, gravityVoltageProxima(), gravityVoltageDistal());

        SmartDashboard.putNumber("Error Prox", errorP);
        SmartDashboard.putNumber("Error Distal", errorD);
    }
    public void voltageMotors(double controlVoltageProxima, double controlVoltageDistal, double gravityVoltageProxima, double gravityVoltageDistal){
        if(getProximalRadians() < boundsProxima[0] && controlVoltageProxima < 0) controlVoltageProxima = 0;
        if(getProximalRadians() > boundsProxima[1] && controlVoltageProxima > 0) controlVoltageProxima = 0;
        if(getDistalRadians() < boundsDistal[0] && controlVoltageDistal < 0) controlVoltageDistal = 0;
        if(getDistalRadians() > boundsDistal[1] && controlVoltageDistal > 0) controlVoltageDistal = 0;
        motorProximal.setVoltage(controlVoltageProxima + gravityVoltageProxima);
        motorDistal.setVoltage(controlVoltageDistal + gravityVoltageDistal);
    }
    
    public double getProximalRadians(){
        double angle = (ext_encProximal.getAbsolutePosition() - 0.94) * (2 * Math.PI) + Math.PI/2;
        if(angle < 0) angle += 2*Math.PI;
        if(angle > 2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }
    public double getDistalRadians(){
        double angle = -(ext_encDistal.getAbsolutePosition() - 0.237) * (2 * Math.PI) - getProximalRadians();
        if(angle < -3.0/2 * Math.PI) angle += 2*Math.PI;
        if(angle > Math.PI/2) angle -= 2*Math.PI;
        return angle;
    }
  @Override
    public void periodic() {
        updateState();


        boolean ShowBounds = true;
        if(ShowBounds){
            SmartDashboard.putBoolean("lb Prox", getProximalRadians() < boundsProxima[0]);
            SmartDashboard.putBoolean("hb Prox", getProximalRadians() > boundsProxima[1]);
            SmartDashboard.putBoolean("lb Dist", getDistalRadians() < boundsDistal[0]);
            SmartDashboard.putBoolean("hb Dist", getDistalRadians() > boundsDistal[1]);
        }
        boolean ShowCurrent = false;
        if(ShowCurrent){
            SmartDashboard.putNumber("Proxima cuurrent", motorProximal.getOutputCurrent());
            SmartDashboard.putNumber("Distal cuurrent", motorDistal.getOutputCurrent());
        }
        
        boolean ShowPositionals = false;
        if(ShowPositionals){
            SmartDashboard.putNumber("Gravity Apply", Math.cos(encoderProximal.getPosition()));
            SmartDashboard.putNumber("Omega Proxima Deg", 180 / Math.PI * encoderProximal.getVelocity());
            SmartDashboard.putNumber("Omega Distal Deg", 180 / Math.PI * encoderDistal.getVelocity());
            SmartDashboard.putNumber("Theta Proxima Deg", 180 / Math.PI * encoderProximal.getPosition());
            SmartDashboard.putNumber("Theta Distal Deg", 180 / Math.PI * encoderDistal.getPosition());
            double[] fK = Arm.forwardKinematics(encoderProximal.getPosition(), encoderDistal.getPosition());
            SmartDashboard.putNumber("pX", fK[0]);
            SmartDashboard.putNumber("pY", fK[1]);
            double[] iK = Arm.inverseKinematics(fK[0], fK[1]);
            SmartDashboard.putNumber("q1", 180 / Math.PI * iK[0]);
            SmartDashboard.putNumber("q2", 180 / Math.PI * iK[1]);
            double[] xdot = Arm.cartesianSpeed();
            SmartDashboard.putNumber("vX", xdot[0]);
            SmartDashboard.putNumber("vY", xdot[1]);
        }
        
        SmartDashboard.putNumber("ExtEnc proximal", getProximalRadians() * 180/Math.PI);
        SmartDashboard.putNumber("ExtEnc distal", getDistalRadians() * 180/Math.PI);
        
    }

@Override
    public void simulationPeriodic() {
// This method will be called once per scheduler run during simulation
    }
}