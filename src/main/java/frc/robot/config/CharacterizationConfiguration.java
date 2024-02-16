// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class CharacterizationConfiguration {

  public Rotation2d frontLeftOffset;
  public Rotation2d frontRightOffset;
  public Rotation2d backLeftOffset;
  public Rotation2d backRightOffset;

  public double driveFeedForwardKs;
  public double driveFeedForwardKv;

  public double wheelRadius;
}
