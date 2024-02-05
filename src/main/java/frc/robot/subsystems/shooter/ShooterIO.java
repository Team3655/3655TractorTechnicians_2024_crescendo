// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double topPositionRad = 0.0;
    public double topVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topCurrentAmps = new double[] {};
    public double topMotorTemp = 0.0;

    public double bottomPositionRad = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomCurrentAmps = new double[] {};
    public double bottomMotorTemp = 0.0;

    public double kickerPositionRad = 0.0;
    public double kickerVelocityRPM = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double[] kickerCurrentAmps = new double[] {};
    public double kickerMotorTemp = 0.0;

    public Rotation2d pivotAbsolutePosition = new Rotation2d();
    public double pivotPositionRotations = 0.0;
    public double pivotVelocityRPM = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps = new double[] {};
    public double pivotMotorTemp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPM, double ffVolts) {}

  /** Run closed loop at the specified angle. */
  public default void setAngle(Rotation2d angle, double ffVolts) {}

  /** Run open loop at the specified voltage. */
  public default void setKickerVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configureFlywheelPID(double kP, double kI, double kD) {}

  /** Set position PID constants. */
  public default void configurePivotPID(double kP, double kI, double kD) {}
}
