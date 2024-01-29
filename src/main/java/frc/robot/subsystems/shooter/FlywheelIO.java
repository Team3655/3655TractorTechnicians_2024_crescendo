// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double topPositionRad = 0.0;
    public double topVelocityRadPerSec = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topCurrentAmps = new double[] {};
    public double topMotorTemp = 0.0;

    public double bottomPositionRad = 0.0;
    public double bottomVelocityRadPerSec = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomCurrentAmps = new double[] {};
    public double bottomMotorTemp = 0.0;

    public double kickerPositionRad = 0.0;
    public double kickerVelocityRadPerSec = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double[] kickerCurrentAmps = new double[] {};
    public double kickerMotorTemp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPM, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default void setKickerVoltage(double volts) {}
}
