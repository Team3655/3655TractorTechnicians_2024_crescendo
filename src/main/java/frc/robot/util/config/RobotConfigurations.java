// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class RobotConfigurations {

  public class RoadRunner {

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

    public static final PortConfiguration portConfig = new PortConfiguration();

    private static final Rotation2d FRONT_LEFT_OFFSET = Rotation2d.fromDegrees(-0.626221);
    private static final Rotation2d FRONT_RIGHT_OFFSET = Rotation2d.fromDegrees(-0.357910);
    private static final Rotation2d BACK_LEFT_OFFSET = Rotation2d.fromDegrees(-0.424805);
    private static final Rotation2d BACK_RIGHT_OFFSET = Rotation2d.fromDegrees(-0.589844);

    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);

    public static final CharacterizationConfiguration characterizationConfig =
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

      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
    }
  }

  public class Timmy {
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

    public static final PortConfiguration portConfig = new PortConfiguration();

    private static final Rotation2d FRONT_LEFT_OFFSET = Rotation2d.fromDegrees(-0.626221);
    private static final Rotation2d FRONT_RIGHT_OFFSET = Rotation2d.fromDegrees(-0.357910);
    private static final Rotation2d BACK_LEFT_OFFSET = Rotation2d.fromDegrees(-0.424805);
    private static final Rotation2d BACK_RIGHT_OFFSET = Rotation2d.fromDegrees(-0.589844);

    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);

    public static final CharacterizationConfiguration characterizationConfig =
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

      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
    }
  }
}
