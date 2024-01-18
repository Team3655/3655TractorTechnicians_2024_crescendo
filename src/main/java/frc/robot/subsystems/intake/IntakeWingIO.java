// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IntakeWingIO {

  @AutoLog
  public static class IntakeWingIOInputs {
    public double intakeAngularVelRadsPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeMotorTemp = 0.0;

    public double deployPositionRads = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
    public double deployMotorTemp = 0.0;
  }

  public default void setDeployAngle(Rotation2d angle) {}

  public default void updateInputs(IntakeWingIOInputs inputs) {}

  public default void configureDeployPID(double p, double i, double d) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}
