// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ShooterIOSpark implements ShooterIO {

  private static final double KICKER_GEAR_RATIO = 1;
  private static final double PIVOT_GEAR_RATIO = 100;

  private final CANSparkFlex top;
  private final RelativeEncoder topEncoder;
  private final SparkPIDController topPID;

  private final CANSparkFlex bottom;
  private final RelativeEncoder bottomEncoder;
  private final SparkPIDController bottomPID;

  private final CANSparkMax kicker;
  private final RelativeEncoder kickerEncoder;

  private final CANSparkMax pivot;
  private final RelativeEncoder pivotEncoder;
  private final AbsoluteEncoder pivotAbsolute;
  private final SparkPIDController pivotPID;

  public ShooterIOSpark() {

    // top flywheel
    top = new CANSparkFlex(30, MotorType.kBrushless);

    top.restoreFactoryDefaults();
    top.setCANTimeout(250);
    top.enableVoltageCompensation(12.0);
    top.setSmartCurrentLimit(40);
    top.burnFlash();

    topEncoder = top.getEncoder();
    topPID = top.getPIDController();
    topPID.setOutputRange(0.0, 1.0);

    // bottom flywheel
    bottom = new CANSparkFlex(31, MotorType.kBrushless);

    bottom.restoreFactoryDefaults();
    bottom.setCANTimeout(250);
    bottom.enableVoltageCompensation(12.0);
    bottom.setSmartCurrentLimit(40);
    // invert bottom
    bottom.setInverted(true);
    bottom.burnFlash();

    bottomEncoder = bottom.getEncoder();
    bottomPID = bottom.getPIDController();
    bottomPID.setOutputRange(0.0, 1.0);

    // kicker
    kicker = new CANSparkMax(32, MotorType.kBrushless);

    kicker.restoreFactoryDefaults();
    kicker.setCANTimeout(250);
    kicker.enableVoltageCompensation(12.0);
    kicker.setSmartCurrentLimit(20);
    kicker.burnFlash();

    kickerEncoder = kicker.getEncoder();

    pivot = new CANSparkMax(33, MotorType.kBrushless);

    pivot.restoreFactoryDefaults();
    pivot.setCANTimeout(250);
    pivot.enableVoltageCompensation(12.0);
    pivot.setSmartCurrentLimit(20);
    pivot.burnFlash();

    pivotEncoder = pivot.getEncoder();
    pivotAbsolute = pivot.getAbsoluteEncoder(Type.kDutyCycle);
    pivotPID = pivot.getPIDController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // region: update top inputs
    inputs.topPositionRad = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = top.getAppliedOutput() * top.getBusVoltage();
    inputs.topCurrentAmps = new double[] {top.getOutputCurrent()};
    inputs.topMotorTemp = top.getMotorTemperature();
    // endregion
    // region: update bottom inputs
    inputs.bottomPositionRad = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottom.getAppliedOutput() * bottom.getBusVoltage();
    inputs.bottomCurrentAmps = new double[] {bottom.getOutputCurrent()};
    inputs.bottomMotorTemp = bottom.getMotorTemperature();
    // endregion
    // region: update kicker inputs
    inputs.kickerPositionRad = kickerEncoder.getPosition() / KICKER_GEAR_RATIO;
    inputs.kickerVelocityRPM = kickerEncoder.getVelocity() / KICKER_GEAR_RATIO;
    inputs.kickerAppliedVolts = kicker.getAppliedOutput() * kicker.getBusVoltage();
    inputs.kickerCurrentAmps = new double[] {kicker.getOutputCurrent()};
    inputs.kickerMotorTemp = kicker.getMotorTemperature();
    // endregion
    // region: update pivot inputs
    inputs.pivotAbsolutePosition = Rotation2d.fromRotations(pivotAbsolute.getPosition());
    inputs.pivotPositionRotations = pivotEncoder.getPosition() / PIVOT_GEAR_RATIO;
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = new double[] {pivot.getOutputCurrent()};
    inputs.pivotMotorTemp = pivot.getMotorTemperature();
    // endregion
  }

  @Override
  public void setVoltage(double volts) {
    top.setVoltage(volts);
    bottom.setVoltage(volts);
  }

  @Override
  public void setKickerVoltage(double volts) {
    kicker.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {
    topPID.setReference(velocityRPM, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
    bottomPID.setReference(velocityRPM, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setAngle(Rotation2d angle, double ffVolts) {
    pivotPID.setReference(
        angle.getRotations(), ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    top.stopMotor();
    bottom.stopMotor();
  }

  @Override
  public void configureFlywheelPID(double kP, double kI, double kD) {
    // top PID
    topPID.setP(kP, 0);
    topPID.setI(kI, 0);
    topPID.setD(kD, 0);
    // bottom PID
    bottomPID.setP(kP, 0);
    bottomPID.setI(kI, 0);
    bottomPID.setD(kD, 0);
  }

  @Override
  public void configurePivotPID(double kP, double kI, double kD) {
    pivotPID.setP(kP, 0);
    pivotPID.setI(kI, 0);
    pivotPID.setD(kD, 0);
  }
}
