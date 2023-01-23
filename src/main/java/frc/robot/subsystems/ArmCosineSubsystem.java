// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang. Math;
import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class ArmCosineSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  double topAngle;
  double baseAngle;
  double distance;
  //double initialAngle;
  double velocity; 
  public CANSparkMax baseMotor = new CANSparkMax(Constants.ArmConstants.baseMotorID, MotorType.kBrushless);
  public CANSparkMax topMotor = new CANSparkMax(Constants.ArmConstants.topMotorID, MotorType.kBrushless);
  public RelativeEncoder baseMotorEncoder = baseMotor.getEncoder();
  public RelativeEncoder topMotorEncoder = topMotor.getEncoder();


  public ArmCosineSubsystem(double distance) {
    this.distance = distance;
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

  public double calcBaseAngle() {
    baseAngle = Math.acos((Math.pow(Constants.ArmConstants.BASE_ARM,2) + Math.pow(distance,2) - Math.pow(Constants.ArmConstants.TOP_ARM,2))/2/Constants.ArmConstants.BASE_ARM/distance);
    return baseAngle;
  }
  public double AngleDistance(double initialAngle) {
    baseAngle = calcBaseAngle();
    double baseAngleDist = Math.abs(baseAngle -initialAngle);
    return baseAngleDist;

  }

  public double timeToAng(double velocity, double radius, double initialAngle){
    double BAD = AngleDistance(initialAngle);
    double result = (Math.pow(radius, 2) * BAD)/(velocity);
    return result;

  }

  public double calcTopAngle() {
    return Math.asin(Math.sin(calcBaseAngle())/Constants.ArmConstants.TOP_ARM*distance);
  }

  public void setTopAngle(){

  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
