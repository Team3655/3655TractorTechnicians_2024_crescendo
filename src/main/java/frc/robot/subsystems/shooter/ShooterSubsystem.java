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

  public static final double KICKER_GEAR_RATIO = 5.0 / 1.0;

  // add one to account for the coin paradox
  public static final double PIVOT_TRACK_RATIO = (130.0 / 17.0); // + 1.0;
  public static final double PIVOT_GEARBOX_RATIO = 25.0 / 1.0;
  public static final double PIVOT_GEAR_RATIO = PIVOT_TRACK_RATIO * PIVOT_GEARBOX_RATIO;

  private static final HashMap<Double, Rotation2d> DISTANCE_TO_ANGLE =
      new HashMap<>() {
        {
          // put(4.5604, Rotation2d.fromRotations(0.1626));
          // put(3.9529, Rotation2d.fromRotations(0.1567));
          // put(3.7741, Rotation2d.fromRotations(0.1557));
          // put(3.1782, Rotation2d.fromRotations(0.1539));
          // put(3.0634, Rotation2d.fromRotations(0.1452));
          // put(2.2614, Rotation2d.fromRotations(0.1289));
          // put(1.3632, Rotation2d.fromRotations(0.0805));
          put(4.5866, Rotation2d.fromDegrees(62));
          put(4.3000, Rotation2d.fromDegrees(61));
          put(4.0225, Rotation2d.fromDegrees(60));
          put(3.4130, Rotation2d.fromDegrees(57));
          put(2.7768, Rotation2d.fromDegrees(54));
          put(2.1333, Rotation2d.fromDegrees(45));
          put(1.6488, Rotation2d.fromDegrees(40));
          put(1.3854, Rotation2d.fromDegrees(36));
        }
      };

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // private final double pivotFFKs;
  // private double pivotFFKg;

  private Rotation2d pivotTarget = new Rotation2d();

  /** Creates a new shooter. */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // TODO: tune FF values for shooter
    switch (Constants.currentMode) {
      case REAL:
        io.configureFlywheelPID(0.001, 0.0, 0.0);
        io.configurePivotPID(3.5, 0.0, 0.0);
        // pivotFFKs = 0.03;
        // pivotFFKg = 0.1;
        break;

      case SIM:
        io.configureFlywheelPID(0.0, 0.0, 0.0);
        io.configurePivotPID(0.0, 0.0, 0.0);

        // pivotFFKs = 0.0;
        // pivotFFKg = 0.0;
        break;

      default:
        io.configureFlywheelPID(0.0, 0.0, 0.0);
        io.configurePivotPID(0.0, 0.0, 0.0);
        // pivotFFKs = 0.0;
        // pivotFFKg = 0.0;
        break;
    }

    // subtract ks from kg for quicker tuning
    // pivotFFKg -= pivotFFKs;

    // log gear ratios so that they can be inspected retroactivly
    Logger.recordOutput("Shooter/PIVOT_TRACK_RATIO ", PIVOT_TRACK_RATIO);
    Logger.recordOutput("Shooter/PIVOT_GEARBOX_RATIO", PIVOT_GEARBOX_RATIO);
    Logger.recordOutput("Shooter/PIVOT_GEAR_RATIO", PIVOT_GEAR_RATIO);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // use feed forward to compensate for the force of gravity of the current
    // shooter angle
    // double ffVolts = (pivotFFKg * Math.cos(pivotTarget.getRadians())) +
    // pivotFFKs;
    // Logger.recordOutput("Shooter/Pivot Target", pivotTarget);
    // io.setAngle(pivotTarget, ffVolts);
  }

  public void runVelocity(double rpm) {
    io.setVelocity(rpm, 0);
    Logger.recordOutput("Shooter/Target RPM", rpm);
  }

  public void setKicker(double volts) {
    io.setKickerVoltage(volts);
  }

  public void stopFlywheel() {
    io.stopFlywheel();
  }

  public void setAngle(Rotation2d angle) {
    io.setAngle(angle, 0);
    pivotTarget = angle;
    Logger.recordOutput("Shooter/Pivot Target", pivotTarget);
  }

  public void setShooterAngleFromDist(double distance) {
    Rotation2d angle = getRotationFromDistance(distance);
    setAngle(angle);
  }

  public void jogAngle(double degrees) {
    pivotTarget = Rotation2d.fromDegrees(pivotTarget.getDegrees() + degrees);
    setAngle(pivotTarget);
  }

  /**
   * Uses a distance input to the target to calculate a shooter angle
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

    // The final target angle of the shooter uses the maxBelow angle as a starting
    // point,
    // then adds the angleDelta times the lerp percent
    Rotation2d angle = DISTANCE_TO_ANGLE.get(maxBelow).plus(angleDelta.times(lerpPercent));

    return angle;
  }
}
