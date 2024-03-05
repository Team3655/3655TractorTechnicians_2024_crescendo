// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {

    public double leftPositionRotations = 0.0;
    public double leftVelocityRPM = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};
    public double leftMotorTemp = 0.0;

    public double rightPositionRotations = 0.0;
    public double rightVelocityRPM = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
    public double rightMotorTemp = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setAngle(Rotation2d angle) {}
}
