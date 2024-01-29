// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private static final HashMap<Double, Rotation2d> DISTANCE_TO_ANGLE =
      new HashMap<>() {
        {
          put(1.0, Rotation2d.fromDegrees(90));
          put(2.0, Rotation2d.fromDegrees(60));
          put(3.0, Rotation2d.fromDegrees(30));
          put(4.0, Rotation2d.fromDegrees(15));
          put(5.0, Rotation2d.fromDegrees(0));
        }
      };

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new shooter. */
  public ShooterSubsystem(FlywheelIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(0.001, 0, 0);
        break;

      case SIM:
        io.configurePID(0, 0, 0);
        break;

      case REPLAY:
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void runVelocity(double rpm) {
    io.setVelocity(rpm, 0);
    Logger.recordOutput("Shooter/Target RPM", rpm);
  }

  public void setKicker(double volts) {
    io.setKickerVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  public void setShooterAngleFromDist() {}

  public Rotation2d getRotationFromDistance(double distance) {

    double minimum = Double.MAX_VALUE;
    double minAbove = Double.MAX_VALUE;
    double maxBelow = Double.MIN_VALUE;
    double maximum = Double.MIN_VALUE;

    for (double distKey : DISTANCE_TO_ANGLE.keySet()) {
      // Sets the minimum
      if (distance > distKey && distKey > maxBelow) maxBelow = distKey;

      // Sets the maximum
      if (distance < distKey && distKey < minAbove) minAbove = distKey;

      if (distKey < minimum) minimum = distKey;

      if (distKey > maximum) maximum = distKey;
    }

    double lerpPercent = ((distance - maxBelow) / (minAbove - maxBelow));

    Rotation2d angleDelta = DISTANCE_TO_ANGLE.get(minAbove).minus(DISTANCE_TO_ANGLE.get(maxBelow));

    Rotation2d angle = DISTANCE_TO_ANGLE.get(maxBelow).plus(angleDelta.times(lerpPercent));

    return angle;
  }
}
