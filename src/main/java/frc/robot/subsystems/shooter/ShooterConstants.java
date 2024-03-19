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

  public static final double PIVOT_TRACK_RATIO = (130.0 / 17.0);
  public static final double PIVOT_GEARBOX_RATIO = 25.0 / 1.0;
  public static final double PIVOT_GEAR_RATIO = PIVOT_TRACK_RATIO * PIVOT_GEARBOX_RATIO;

  public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

  static {
    ANGLE_MAP.put(5.1436, 26.3);
    ANGLE_MAP.put(4.9348, 26.3);
    ANGLE_MAP.put(4.6436, 28.0);
    ANGLE_MAP.put(4.5866, 28.0);
    ANGLE_MAP.put(4.3000, 28.0);
    ANGLE_MAP.put(4.0225, 29.0);
    ANGLE_MAP.put(3.4130, 33.0);
    ANGLE_MAP.put(2.7768, 36.0);
    ANGLE_MAP.put(2.1333, 45.0);
    ANGLE_MAP.put(1.6488, 50.0);
    ANGLE_MAP.put(1.3854, 54.0);
  }

  public static class ShooterTargets {

    public static final int SPEAKER_RPM = 5000;

    public static final ShooterState IDLE =
        new ShooterState(Rotation2d.fromDegrees(65.0), Optional.empty());

    public static final ShooterState DEFAULT_STOP =
        new ShooterState(Rotation2d.fromDegrees(65.0), Optional.of(0));

    public static final ShooterState SUBWOFER =
        new ShooterState(Rotation2d.fromDegrees(60.0), Optional.of(SPEAKER_RPM));

    public static final ShooterState AMP =
        new ShooterState(Rotation2d.fromDegrees(52), Optional.of(2000));

    public static final ShooterState BABY_BIRD =
        new ShooterState(Rotation2d.fromDegrees(70), Optional.of(-1500));

    public static final ShooterState INDEX =
        new ShooterState(Rotation2d.fromDegrees(45), Optional.empty());

    public static final ShooterState LEAVE_LINE =
        new ShooterState(Rotation2d.fromDegrees(42), Optional.of(SPEAKER_RPM));

    public static final ShooterState BEHIND_PODIUM =
        new ShooterState(Rotation2d.fromDegrees(33.5), Optional.of(SPEAKER_RPM));

    public static final ShooterState PREP_CLIMB =
        new ShooterState(Rotation2d.fromDegrees(0.0), Optional.empty());

    public static final ShooterState CLIMB_STAGE_ONE =
        new ShooterState(Rotation2d.fromDegrees(0.0), Optional.empty());

    public static final ShooterState CLIMBE_STAGE_TWO =
        new ShooterState(Rotation2d.fromDegrees(0.0), Optional.empty());

    public static final ShooterState TRAP =
        new ShooterState(Rotation2d.fromDegrees(60), Optional.of(SPEAKER_RPM));
  }

  /**
   * @param angle A Rotation2d, use Rotation2d.fromdegrees(88.75 - 24.5)
   */
  public record ShooterState(Rotation2d angle, Optional<Integer> rpm) {}
}
