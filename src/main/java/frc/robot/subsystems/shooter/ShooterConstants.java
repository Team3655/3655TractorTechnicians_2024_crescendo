// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Optional;

/** Add your docs here. */
public class ShooterConstants {

  public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(88.75);
  public static final Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(24.5);
  public static final Rotation2d PIVOT_TARGETING_OFFSET = Rotation2d.fromDegrees(0.0);

  public static final double PIVOT_TRACK_RATIO = 130.0 / 17.0;
  public static final double PIVOT_GEARBOX_RATIO = 25.0 / 1.0;
  public static final double PIVOT_GEAR_RATIO = PIVOT_TRACK_RATIO * PIVOT_GEARBOX_RATIO;

  public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

  public enum ShooterState {
    TRACKING,
    DEAD_RECKONING,
    TUNING,
  }

  static {
    ANGLE_MAP.put(7.2500, 29.0);
    ANGLE_MAP.put(7.0000, 29.0);
    ANGLE_MAP.put(6.5000, 29.0);
    ANGLE_MAP.put(6.0000, 29.6);
    ANGLE_MAP.put(5.7500, 29.9);
    ANGLE_MAP.put(5.5000, 30.3);
    ANGLE_MAP.put(5.0000, 31.0);
    ANGLE_MAP.put(4.5000, 32.0);
    ANGLE_MAP.put(4.0000, 34.0);
    ANGLE_MAP.put(3.5000, 36.0);
    ANGLE_MAP.put(3.0000, 39.5);
    ANGLE_MAP.put(2.5000, 42.0);
    ANGLE_MAP.put(2.0000, 49.0);
    ANGLE_MAP.put(1.7500, 53.0);
    ANGLE_MAP.put(1.5000, 58.0);
    ANGLE_MAP.put(1.3854, 63.0);
  }

  public static class ShooterTargets {

    public static final double SPEAKER_RPM = 6000.0;

    public static final ShooterTarget IDLE =
        new ShooterTarget(Rotation2d.fromDegrees(70.0), Optional.empty());

    public static final ShooterTarget DEFAULT_STOP =
        new ShooterTarget(Rotation2d.fromDegrees(70.0), Optional.of(0.0));

    public static final ShooterTarget SUBWOFER =
        new ShooterTarget(Rotation2d.fromDegrees(60.0), Optional.of(SPEAKER_RPM));

    public static final ShooterTarget AMP =
        new ShooterTarget(Rotation2d.fromDegrees(57), Optional.of(1300.0));

    public static final ShooterTarget BABY_BIRD =
        new ShooterTarget(Rotation2d.fromDegrees(70), Optional.of(-2500.0));

    public static final ShooterTarget INDEX =
        new ShooterTarget(Rotation2d.fromDegrees(45), Optional.empty());

    public static final ShooterTarget LEAVE_LINE =
        new ShooterTarget(Rotation2d.fromDegrees(42), Optional.of(SPEAKER_RPM));

    public static final ShooterTarget BEHIND_PODIUM =
        new ShooterTarget(Rotation2d.fromDegrees(33.5), Optional.of(SPEAKER_RPM));

    public static final ShooterTarget AUTO_LONG =
        new ShooterTarget(Rotation2d.fromDegrees(29), Optional.of(SPEAKER_RPM));

    public static final ShooterTarget PREP_CLIMB =
        new ShooterTarget(Rotation2d.fromDegrees(88.75), Optional.empty());

    public static final ShooterTarget CLIMB_STAGE_ONE =
        new ShooterTarget(Rotation2d.fromDegrees(45.0), Optional.empty());

    public static final ShooterTarget CLIMBE_STAGE_TWO =
        new ShooterTarget(Rotation2d.fromDegrees(70.0), Optional.empty());

    public static final ShooterTarget TRAP =
        new ShooterTarget(Rotation2d.fromDegrees(43), Optional.of(5500.0));

    public static final ShooterTarget PASS =
        new ShooterTarget(Rotation2d.fromDegrees(55), Optional.of(4000.0));

    public static final ShooterTarget SPIT =
        new ShooterTarget(Rotation2d.fromDegrees(30), Optional.of(3000.0));
  }

  /**
   * @param angle A Rotation2d, use Rotation2d.fromdegrees(88.75 - 24.5)
   * @param rpm An Optional<Integer> for the shooter rpm, use Optional.empty() for 0 volts
   */
  public record ShooterTarget(Rotation2d angle, Optional<Double> rpm) {}
}
