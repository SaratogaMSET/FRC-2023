package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        motorDistal.setInverted(true);
    }

    ArmKinematics Arm = new ArmKinematics(0, 0);
    public CANSparkMax motorProximal = new CANSparkMax(Constants.Arm.proximalMotorID, MotorType.kBrushless);
    public CANSparkMax motorDistal = new CANSparkMax(Constants.Arm.distalMotorID, MotorType.kBrushless);
    SparkMaxAbsoluteEncoder encoderProximal = motorProximal.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(0));
    SparkMaxAbsoluteEncoder encoderDistal = motorProximal.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(0));

    public void calibrateArm(){
        double convertRotRad = 2 * Math.PI;
        encoderProximal.setPositionConversionFactor(convertRotRad/Constants.Arm.gearProximalReduction);
        encoderProximal.setVelocityConversionFactor(convertRotRad/Constants.Arm.gearProximalReduction);
        encoderDistal.setPositionConversionFactor(convertRotRad/Constants.Arm.gearProximalReduction);
        encoderDistal.setVelocityConversionFactor(convertRotRad/Constants.Arm.gearProximalReduction);
        encoderProximal.setZeroOffset(Units.degreesToRadians(Constants.Arm.initialAngleProximal));
        encoderDistal.setZeroOffset(Units.degreesToRadians(Constants.Arm.initialAngleDistal));
    }
    public void updateState(){
        Arm.setAngles(encoderProximal.getPosition(), encoderDistal.getPosition());
        Arm.setOmegas(encoderProximal.getVelocity(), encoderDistal.getVelocity());
    }
    public void voltageMotors(double voltageProximal, double voltageDistal){
        motorProximal.setVoltage(voltageProximal);
        motorDistal.setVoltage(voltageDistal);
    }
    public void armControl(double px, double py){
        double[] torques = Arm.torquePID(px, py, 0, 0);

        double voltageProximal = VoltageControl.GearedNEOVoltage(torques[0], encoderProximal.getVelocity(), 0.1, Constants.Arm.gearProximalReduction);
        double voltageDistal = VoltageControl.GearedNEOVoltage(torques[0], encoderDistal.getVelocity(), 0.1, Constants.Arm.gearDistalReduction);
        motorProximal.setVoltage(voltageProximal);
        motorDistal.setVoltage(voltageDistal);
    }

  @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle Proxima", 180 / Math.PI * encoderProximal.getPosition());
        SmartDashboard.putNumber("Angle Distal", 180 / Math.PI * encoderDistal.getPosition());
    }

@Override
    public void simulationPeriodic() {
// This method will be called once per scheduler run during simulation
    }
}