// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotVersion;

/** Add your docs here. */
public class RobotConfigurations {

  private static RoadRunner roadRunner = new RoadRunner();
  private static Timmy timmy = new Timmy();

  public static PortConfiguration getPortConfiguration(RobotVersion version) {
    switch (version) {
      case ROADRUNNER:
        return roadRunner.portConfig;

      case TIMMY:
        return timmy.portConfig;

      default:
        throw new IllegalArgumentException(
            "Current robot config is not accounted for in the switch statement!");
    }
  }

  public static CharacterizationConfiguration getCharacterizationConfiguration(
      RobotVersion version) {
    switch (version) {
      case ROADRUNNER:
        return roadRunner.characterizationConfig;

      case TIMMY:
        return timmy.characterizationConfig;

      default:
        throw new IllegalArgumentException(
            "Current robot config is not accounted for in the switch statement!");
    }
  }

  public static class RoadRunner {

    private static final int GYRO_ID = 20;

    private static final int FRONT_LEFT_TURN_ID = 1;
    private static final int FRONT_LEFT_DRIVE_ID = 2;
    private static final int FRONT_LEFT_ABSOLUTE_ID = 3;

    private static final int FRONT_RIGHT_TURN_ID = 4;
    private static final int FRONT_RIGHT_DRIVE_ID = 5;
    private static final int FRONT_RIGHT_ABSOLUTE_ID = 6;

    private static final int BACK_LEFT_TURN_ID = 7;
    private static final int BACK_LEFT_DRIVE_ID = 8;
    private static final int BACK_LEFT_ABSOLUTE_ID = 9;

    private static final int BACK_RIGHT_TURN_ID = 10;
    private static final int BACK_RIGHT_DRIVE_ID = 11;
    private static final int BACK_RIGHT_ABSOLUTE_ID = 12;

    private static final int TOP_FLYWHEEL_ID = 30;
    private static final int BOTTOM_FLYWHEEL_ID = 31;
    private static final int KICKER_ID = 32;
    private static final int PIVOT_ID = 33;
    private static final int KICKER_BEAMBREAK_PORT = 1;
    private static final int FLYWHEEL_BEAMBREAK_PORT = 2;

    private static final int INTAKE_MOTOR_ID = 40;
    private static final int DEPLOY_SOLENOID_PORT = 1;
    private static final int INTAKE_BEAMBREAK_PORT = 0;

    private static final int RIGHT_CLIMER_ID = 41;
    private static final int LEFT_CLIMER_ID = 42;

    private static final int PNEUMATIC_HUB_ID = 50;

    private static final String LIMELIGHT_NAME = "limelight";

    private static final boolean USE_PHEONIX_PRO = true;
    private static final String DRIVE_CANBUS = "ctre";

    public final PortConfiguration portConfig = new PortConfiguration();

    private static final double FRONT_LEFT_OFFSET = -0.626221;
    private static final double FRONT_RIGHT_OFFSET = -0.357910;
    private static final double BACK_LEFT_OFFSET = -0.424805;
    private static final double BACK_RIGHT_OFFSET = -0.589844;

    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);
    private static final double MAX_VELOCITY = Units.feetToMeters(19.5);

    public final CharacterizationConfiguration characterizationConfig =
        new CharacterizationConfiguration();

    public RoadRunner() {
      portConfig.gyroID = GYRO_ID;

      portConfig.frontLeftTurnMotorID = FRONT_LEFT_TURN_ID;
      portConfig.frontLeftDriveMotorID = FRONT_LEFT_DRIVE_ID;
      portConfig.frontLeftAbsoluteEncoderID = FRONT_LEFT_ABSOLUTE_ID;

      portConfig.frontRightTurnMotorID = FRONT_RIGHT_TURN_ID;
      portConfig.frontRightDriveMotorID = FRONT_RIGHT_DRIVE_ID;
      portConfig.frontRightAbsoluteEncoderID = FRONT_RIGHT_ABSOLUTE_ID;

      portConfig.backLeftTurnMotorID = BACK_LEFT_TURN_ID;
      portConfig.backLeftDriveMotorID = BACK_LEFT_DRIVE_ID;
      portConfig.backLeftAbsoluteEncoderID = BACK_LEFT_ABSOLUTE_ID;

      portConfig.backRightTurnMotorID = BACK_RIGHT_TURN_ID;
      portConfig.backRightDriveMotorID = BACK_RIGHT_DRIVE_ID;
      portConfig.backRightAbsoluteEncoderID = BACK_RIGHT_ABSOLUTE_ID;

      portConfig.topFlywheelMoterID = TOP_FLYWHEEL_ID;
      portConfig.bottomFlywheelMotorID = BOTTOM_FLYWHEEL_ID;
      portConfig.kickerMotorID = KICKER_ID;
      portConfig.pivotMotorID = PIVOT_ID;
      portConfig.kickerBeamBreakPort = KICKER_BEAMBREAK_PORT;
      portConfig.flyWheelBeamBreakport = FLYWHEEL_BEAMBREAK_PORT;

      portConfig.intakeMotorID = INTAKE_MOTOR_ID;
      portConfig.deploySolenoidPort = DEPLOY_SOLENOID_PORT;
      portConfig.intakeBeamBreakPort = INTAKE_BEAMBREAK_PORT;

      portConfig.rightClimberMotorID = RIGHT_CLIMER_ID;
      portConfig.leftClimberMotorID = LEFT_CLIMER_ID;

      portConfig.pneumaticHubID = PNEUMATIC_HUB_ID;

      portConfig.limelightName = LIMELIGHT_NAME;

      portConfig.usePheonixPro = USE_PHEONIX_PRO;
      portConfig.driveCANBus = DRIVE_CANBUS;

      characterizationConfig.frontLeftOffset = FRONT_LEFT_OFFSET;
      characterizationConfig.frontRightOffset = FRONT_RIGHT_OFFSET;
      characterizationConfig.backLeftOffset = BACK_LEFT_OFFSET;
      characterizationConfig.backRightOffset = BACK_RIGHT_OFFSET;

      characterizationConfig.driveGearRatio = DRIVE_GEAR_RATIO;
      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
      characterizationConfig.maxVelocity = MAX_VELOCITY;
    }
  }

  public static class Timmy {
    private static final int GYRO_ID = 20;

    private static final int FRONT_LEFT_TURN_ID = 1;
    private static final int FRONT_LEFT_DRIVE_ID = 2;
    private static final int FRONT_LEFT_ABSOLUTE_ID = 3;

    private static final int FRONT_RIGHT_TURN_ID = 4;
    private static final int FRONT_RIGHT_DRIVE_ID = 5;
    private static final int FRONT_RIGHT_ABSOLUTE_ID = 6;

    private static final int BACK_LEFT_TURN_ID = 7;
    private static final int BACK_LEFT_DRIVE_ID = 8;
    private static final int BACK_LEFT_ABSOLUTE_ID = 9;

    private static final int BACK_RIGHT_TURN_ID = 10;
    private static final int BACK_RIGHT_DRIVE_ID = 11;
    private static final int BACK_RIGHT_ABSOLUTE_ID = 12;

    private static final int TOP_FLYWHEEL_ID = 30;
    private static final int BOTTOM_FLYWHEEL_ID = 31;
    private static final int KICKER_ID = 32;
    private static final int PIVOT_ID = 33;
    private static final int KICKER_BEAMBREAK_PORT = 1;
    private static final int FLYWHEEL_BEAMBREAK_PORT = 2;

    private static final int INTAKE_MOTOR_ID = 40;
    private static final int DEPLOY_SOLENOID_PORT = 1;
    private static final int INTAKE_BEAMBREAK_PORT = 0;

    private static final int RIGHT_CLIMER_ID = 41;
    private static final int LEFT_CLIMER_ID = 42;

    private static final int PNEUMATIC_HUB_ID = 50;

    private static final String LIMELIGHT_NAME = "limelight";

    private static final boolean USE_PHEONIX_PRO = true;
    private static final String DRIVE_CANBUS = "ctre";

    public final PortConfiguration portConfig = new PortConfiguration();

    private static final double FRONT_LEFT_OFFSET = -0.626221;
    private static final double FRONT_RIGHT_OFFSET = -0.357910;
    private static final double BACK_LEFT_OFFSET = -0.424805;
    private static final double BACK_RIGHT_OFFSET = -0.589844;

    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);
    private static final double MAX_VELOCITY = Units.feetToMeters(15.5);

    public final CharacterizationConfiguration characterizationConfig =
        new CharacterizationConfiguration();

    public Timmy() {
      portConfig.gyroID = GYRO_ID;

      portConfig.frontLeftTurnMotorID = FRONT_LEFT_TURN_ID;
      portConfig.frontLeftDriveMotorID = FRONT_LEFT_DRIVE_ID;
      portConfig.frontLeftAbsoluteEncoderID = FRONT_LEFT_ABSOLUTE_ID;

      portConfig.frontRightTurnMotorID = FRONT_RIGHT_TURN_ID;
      portConfig.frontRightDriveMotorID = FRONT_RIGHT_DRIVE_ID;
      portConfig.frontRightAbsoluteEncoderID = FRONT_RIGHT_ABSOLUTE_ID;

      portConfig.backLeftTurnMotorID = BACK_LEFT_TURN_ID;
      portConfig.backLeftDriveMotorID = BACK_LEFT_DRIVE_ID;
      portConfig.backLeftAbsoluteEncoderID = BACK_LEFT_ABSOLUTE_ID;

      portConfig.backRightTurnMotorID = BACK_RIGHT_TURN_ID;
      portConfig.backRightDriveMotorID = BACK_RIGHT_DRIVE_ID;
      portConfig.backRightAbsoluteEncoderID = BACK_RIGHT_ABSOLUTE_ID;

      portConfig.topFlywheelMoterID = TOP_FLYWHEEL_ID;
      portConfig.bottomFlywheelMotorID = BOTTOM_FLYWHEEL_ID;
      portConfig.kickerMotorID = KICKER_ID;
      portConfig.pivotMotorID = PIVOT_ID;
      portConfig.kickerBeamBreakPort = KICKER_BEAMBREAK_PORT;
      portConfig.flyWheelBeamBreakport = FLYWHEEL_BEAMBREAK_PORT;

      portConfig.intakeMotorID = INTAKE_MOTOR_ID;
      portConfig.deploySolenoidPort = DEPLOY_SOLENOID_PORT;
      portConfig.intakeBeamBreakPort = INTAKE_BEAMBREAK_PORT;

      portConfig.rightClimberMotorID = RIGHT_CLIMER_ID;
      portConfig.leftClimberMotorID = LEFT_CLIMER_ID;

      portConfig.pneumaticHubID = PNEUMATIC_HUB_ID;

      portConfig.limelightName = LIMELIGHT_NAME;

      portConfig.usePheonixPro = USE_PHEONIX_PRO;
      portConfig.driveCANBus = DRIVE_CANBUS;

      characterizationConfig.frontLeftOffset = FRONT_LEFT_OFFSET;
      characterizationConfig.frontRightOffset = FRONT_RIGHT_OFFSET;
      characterizationConfig.backLeftOffset = BACK_LEFT_OFFSET;
      characterizationConfig.backRightOffset = BACK_RIGHT_OFFSET;

      characterizationConfig.driveGearRatio = DRIVE_GEAR_RATIO;
      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
      characterizationConfig.maxVelocity = MAX_VELOCITY;
    }
  }
}
