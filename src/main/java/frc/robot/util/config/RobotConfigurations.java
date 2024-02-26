// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/** Add your docs here. */
public class RobotConfigurations {

  public class RoadRunner {

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
    
    

    public PortConfiguration portConfig = new PortConfiguration();

    public RoadRunner() {
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
    }
  }
}
