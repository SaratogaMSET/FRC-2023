// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang. Math;
import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class ArmCosineSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

 
  PIDController controller;
  //double initialAngle;
  double targetVelocity;

  public CANSparkMax baseMotor;
  public CANSparkMax topMotor;
  public RelativeEncoder baseMotorEncoder;
  public RelativeEncoder topMotorEncoder;


  public ArmCosineSubsystem(double targetVelocity) {
    this.targetVelocity = targetVelocity;
    
    controller = new PIDController(0.3, 0, 0);
    baseMotor = new CANSparkMax(Constants.ArmConstants.baseMotorID, MotorType.kBrushless);
    topMotor = new CANSparkMax(Constants.ArmConstants.topMotorID, MotorType.kBrushless);
    baseMotorEncoder = baseMotor.getEncoder();
    topMotorEncoder = topMotor.getEncoder();


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void moveToAngle(double distance){

    double baseAngle = Math.acos((Math.pow(Constants.ArmConstants.BASE_ARM,2) + Math.pow(distance,2) - Math.pow(Constants.ArmConstants.TOP_ARM,2))/2/Constants.ArmConstants.BASE_ARM/distance); //in radians

    double topAngle = Math.asin(Math.sin(baseAngle)/Constants.ArmConstants.TOP_ARM*distance); //in radians

    controller.setTolerance(0.1); /*CHANGE NUMBER*/
    double baseOutput = controller.calculate(baseMotorEncoder.getPosition(), baseAngle); // change angleRadians to rotations via doc
    double topOutput = controller.calculate(topMotorEncoder.getPosition(), topAngle); // change angleRadians to rotations via doc

    baseMotor.set(baseOutput * targetVelocity);
    topMotor.set(topOutput * targetVelocity);

  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
