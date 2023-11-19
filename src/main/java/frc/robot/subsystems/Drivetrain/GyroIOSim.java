// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  double heading = 0.0;

  @Override
  public GyroIOInputsAutoLogged updateInputs() {
    var inputs = new GyroIOInputsAutoLogged();
    inputs.headingDegrees = heading;
    return inputs;
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(heading);
  }
  @Override
  public void resetHeading(){}


@Override
public Rotation2d getRoll(){
    return Rotation2d.fromDegrees(0.0);
}

@Override
public Rotation2d getPitch(){
   return  Rotation2d.fromDegrees(0.0);
}

@Override
public double getWorldLinearAccelX(){
    return 0.0;
}
}
