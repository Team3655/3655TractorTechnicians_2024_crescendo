// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/** Add your docs here. */
public class PortConfiguration {

  // region: drivetrain
  public int frontLeftDriveMotorID;
  public int frontLeftTurnMotorID;
  public int frontLeftAbsoluteEncoderID;

  public int frontRightDriveMotorID;
  public int frontRightTurnMotorID;
  public int frontRightAbsoluteEncoderID;

  public int backLeftDriveMotorID;
  public int backLeftTurnMotorID;
  public int backLeftAbsoluteEncoderID;

  public int backRightDriveMotorID;
  public int backRightTurnMotorID;
  public int backRightAbsoluteEncoderID;
  // endregion

  // region: shooter
  public int topFlywheelMotrID;
  public int bottomFlywheelMotorID;
  public int kickerMotorID;
  public int pivotMotorID;
  public int kickerBeamBreakPort;
  public int flyWheelBeamBreakport;
  // endregion

  // region: intake
  public int intakeMotorID;
  public int deploySolenoidPort;
  public int intakeBeamBreakPort;
  // endregion

  // region: climber
  public int rightClimberMotorID;
  public int leftClimberMotorID;
  // endregion

  public int pneumaticHubID;

  public boolean usePheonixPro;
  public String driveCANBus;
}
