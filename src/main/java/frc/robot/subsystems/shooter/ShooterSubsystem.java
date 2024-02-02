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

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Creates a new shooter. */
  public ShooterSubsystem(ShooterIO io) {
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
    // io.updateInputs(inputs);
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

  public void setShooterAngleFromDist(double distance) {
    Rotation2d angle = getRotationFromDistance(distance);
    // TODO: add pivot to io
  }

  /**
   * Uses a distance to the target to calculate a shooter angle
   *
   * @param distance the delta from target to robot
   * @return a Rotation2d represent the optimal shooter angle
   */
  public Rotation2d getRotationFromDistance(double distance) {

    // Use max/min value to ensure variable gets a value on first loop cycle
    double minimum = Double.MAX_VALUE;
    double minAbove = Double.MAX_VALUE;
    double maxBelow = Double.MIN_VALUE;
    double maximum = Double.MIN_VALUE;

    for (double distKey : DISTANCE_TO_ANGLE.keySet()) {
      // finds the key key below the input
      if (distance > distKey && distKey > maxBelow) maxBelow = distKey;

      // finds the closest key above the input
      if (distance < distKey && distKey < minAbove) minAbove = distKey;

      // finds the maximum
      if (distKey < minimum) minimum = distKey;

      // finds the minimum
      if (distKey > maximum) maximum = distKey;
    }

    // if the input is outside the range of the map return
    // the minimum or maximum angle
    if (distance > maximum) return DISTANCE_TO_ANGLE.get(maximum);
    else if (distance < minimum) return DISTANCE_TO_ANGLE.get(minimum);

    // how close the input is to minAbove expressed as a percentage of the delta
    // between maxBelow and minAbove
    double lerpPercent = ((distance - maxBelow) / (minAbove - maxBelow));

    // the delta between the rotations associated with minAbove and minBelow
    Rotation2d angleDelta = DISTANCE_TO_ANGLE.get(minAbove).minus(DISTANCE_TO_ANGLE.get(maxBelow));

    // The final target angle of the shooter uses the maxBelow angle as a starting point,
    // then adds the angleDelta times the lerp percent
    Rotation2d angle = DISTANCE_TO_ANGLE.get(maxBelow).plus(angleDelta.times(lerpPercent));

    return angle;
  }
}
