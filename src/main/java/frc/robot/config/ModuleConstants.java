package frc.robot.config;

import lombok.Builder;
import lombok.Getter;

@Builder
public class ModuleConstants {

  public enum DriveRatio {
    LL1,
    LL2,
    LL3,
    LL4
  }

  private final DriveRatio driveRatio;
  private final double shaftPinion;
  
  @Getter
  private final int driveMotorCANID;
  @Getter
  private final int steerMotorCANID;
  @Getter
  private final int cancoderCANID;

  @Getter
  private final double offsetRotations;
  
  public double getGearRatio() {
    switch (driveRatio) {
      case LL1:
        return (50.0 / shaftPinion) * (19.0 / 25.0) * (45.0 / 15.0);

      case LL2:
        return (50.0 / shaftPinion) * (17.0 / 27.0) * (45.0 / 15.0);

      case LL3:
        return (50.0 / shaftPinion) * (16.0 / 28.0) * (45.0 / 15.0);

      case LL4:
        return (48.0 / shaftPinion) * (16.0 / 28.0) * (45.0 / 15.0);

      default:
        return 0.0;
    }
  }

}
