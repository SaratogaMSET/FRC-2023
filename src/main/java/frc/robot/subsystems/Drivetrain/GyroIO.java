// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double headingDegrees;
  }

  public abstract GyroIOInputsAutoLogged updateInputs();

  public abstract Rotation2d getHeading();

  public abstract Rotation2d getRoll();

  public abstract Rotation2d getPitch();

  public abstract double getWorldLinearAccelX();

  public abstract void resetHeading();
}