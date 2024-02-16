// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelRadsPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double intakeMotorTemp = 0.0;

    public boolean deploySolenoidState = false;
    public double deployPositionRads = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setLinear(Boolean value) {}

  public default void suckUpNote(Boolean value, int volts) {}
}
