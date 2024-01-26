// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IndexIO {

  public class IndexIOInputs {

    double indexPositionRads = 0.0;
    double indexVelRadsPerSec = 0.0;
    double indexAppliedVolts = 0.0;
    double[] indexCurrentAmps = new double[] {};
    double indexMotorTemp = 0.0;

    Rotation2d pivotPosition = new Rotation2d();
    double pivotVelRadsPerSec = 0.0;
    double pivotAppliedVolts = 0.0;
    double[] pivotCurrentAmps = new double[] {};
    double pivotMotorTemp = 0.0;
  }

  public default void updateInputs(IndexIOInputs inputs) {}
  ;

  public default void setIndexVoltage(double volts) {}
  ;

  public default void setPivotAngle(Rotation2d angle) {}
  ;
}
