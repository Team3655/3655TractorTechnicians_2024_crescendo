// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public static final double SUCKER_GEAR_RATIO = 1.5;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static final IntakeState TUCKED_STATE = new IntakeState(0, true);
  public static final IntakeState INTAKE_STATE = new IntakeState(1, false);

  private IntakeState targetState = TUCKED_STATE;

  /** Creates a new intake. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setLinearPosition(Boolean value) {
    io.setLinear(value);
  }

  public void setIntakeState(IntakeState state) {
    targetState = state;
    io.setVoltage(targetState.outputPercent * 12.0);

    Logger.recordOutput("Intake/Sucker", targetState.outputPercent);
    Logger.recordOutput("Intake/isDeployed", targetState.isDeployed);
  }

  public void setIntakeVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void suckUpNote(Boolean value, int volts) {
    io.suckUpNote(value, volts);
  }

  /** A class with two finals variables that control the intake's state */
  public static class IntakeState {
    public final boolean isDeployed;
    public final double outputPercent;

    /** Creates a new IntakeState with default values of (0, 0) */
    public IntakeState() {
      this(0, false);
    }

    /**
     * Creates a new IntakeState with values input by the user
     *
     * @param outputPercent The output percent you want the intake to run at
     * @param angle A Rotation2d representing the tartget angle of the intake, with positive being
     *     forward away from the robot
     */
    public IntakeState(double outputPercent, boolean isDeployed) {
      this.isDeployed = isDeployed;
      this.outputPercent = outputPercent;
    }
  }
}
