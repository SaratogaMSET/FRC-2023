// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class GyroIONavx implements GyroIO {
  public final AHRS m_navx;

  public GyroIONavx() {
    m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    resetHeading();
  }

  @Override
  public GyroIOInputsAutoLogged updateInputs() {
    var inputs = new GyroIOInputsAutoLogged();
    inputs.headingDegrees = m_navx.getYaw();
    return inputs;
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_navx.getYaw());
  }

  @Override
  public Rotation2d getRoll() {
    return new Rotation2d(m_navx.getRoll());
  }

  @Override
  public Rotation2d getPitch() {
    return new Rotation2d(m_navx.getPitch());
  }

  @Override
  public double getWorldLinearAccelX() {
    return m_navx.getWorldLinearAccelX();
  }

  @Override
  public void resetHeading(){
    m_navx.zeroYaw();
  }
}