// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    IDLE,
    STAGE_ONE,
    FULL_DEPLOY,
    FORWARD_FEED,
    REVERSE_FEED,
    INDEX
  }

  public static final double INTAKE_GEAR_RATIO = 3.0;
  public static final double FEEDER_GEAR_RATIO = 5.0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final LinearFilter lowpass = LinearFilter.movingAverage(35);

  private final LoggedDashboardBoolean hasPiece;

  private IntakeState currentState = IntakeState.IDLE;

  private double indexDistanceAverage;

  /** Creates a new intake. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    hasPiece = new LoggedDashboardBoolean("ShooterIntake: Has Piece?");
    hasPiece.setDefault(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    indexDistanceAverage = lowpass.calculate(inputs.indexDistanceMM);
    Logger.recordOutput("Intake/Index Distance Average", indexDistanceAverage);

    hasPiece.set(getProximity() || (getIndexDistanceMM() <= 335));

    switch (currentState) {
      case IDLE -> {
        io.setStageOneValue(Value.kReverse);
        io.setStageTwoValue(Value.kReverse);
        io.setFeederVoltage(0.0);
        io.setIntakeVoltage(0.0);
      }
      case STAGE_ONE -> {
        io.setStageOneValue(Value.kForward);
        io.setStageTwoValue(Value.kReverse);
        io.setFeederVoltage(0.0);
        io.setIntakeVoltage(0.0);
      }
      case FULL_DEPLOY -> {
        io.setStageOneValue(Value.kForward);
        io.setStageTwoValue(Value.kForward);
        io.setFeederVoltage(0.0);
        io.setIntakeVoltage(10.0);
      }
      case FORWARD_FEED -> {
        io.setStageOneValue(Value.kReverse);
        io.setStageTwoValue(Value.kReverse);
        io.setFeederVoltage(12.0);
        io.setIntakeVoltage(12.0);
      }
      case REVERSE_FEED -> {
        io.setStageOneValue(Value.kReverse);
        io.setStageTwoValue(Value.kReverse);
        io.setFeederVoltage(-12.0);
        io.setIntakeVoltage(-10.0);
      }
      case INDEX -> {
        io.setStageOneValue(Value.kReverse);
        io.setStageTwoValue(Value.kReverse);
        io.setFeederVoltage(5.0);
        io.setIntakeVoltage(5.0);
      }
    }
  }

  public void setState(IntakeState state) {
    currentState = state;
  }

  public IntakeState getState() {
    return currentState;
  }

  public boolean getProximity() {
    return inputs.hasPiece;
  }

  public double getIndexDistanceMM() {
    return indexDistanceAverage;
  }
}
