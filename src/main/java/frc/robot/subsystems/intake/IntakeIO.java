// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelRadsPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double intakeMotorTemp = 0.0;

    public double feedVelRadsPerSec = 0.0;
    public double feedAppliedVolts = 0.0;
    public double[] feedCurrentAmps = new double[] {};
    public double feedMotorTemp = 0.0;

    public boolean indexDistanceConnected = false;
    public double indexDistanceMM = 0.0;
    public double indexDistanceAmbient = 0.0;

    public boolean hasPiece = false;

    public double compressorPressure = 0.0;
    public double compressorCurrent = 0.0;
    public double pneumaticHubVoltage = 0.0;
    public boolean compressorEnabled = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVoltage(double volts) {}

  public default void setFeederVoltage(double volts) {}

  public default void setDeployed(Boolean deployed) {}

  public default void setStageOneValue(Value value) {}

  public default void setStageTwoValue(Value value) {}

  public default boolean getProximity() {
    return true;
  }

  public default double getPressure() {
    return 0;
  }
}
