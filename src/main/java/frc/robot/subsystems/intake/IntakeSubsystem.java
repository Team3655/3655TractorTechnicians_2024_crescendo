// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public static final double SUCKER_GEAR_RATIO = 3.0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static final IntakeState TUCKED_STATE = new IntakeState(0.0, false);
  public static final IntakeState INTAKE_STATE = new IntakeState(10.0, true);

  private IntakeState targetState = TUCKED_STATE;

  /** Creates a new intake. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    // tell the intake to tuck when the proxy sensor is tripped
    Trigger photoEye = new Trigger(() -> io.getProximity());
    photoEye.onTrue(Commands.runOnce(() -> setState(TUCKED_STATE), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Proximity", io.getProximity());
    Logger.recordOutput("Pressure", io.getPressure());
  }

  /**
   * Sets the intake to the specified state
   *
   * @param state the state to set the intake to
   *     <p>should be accessed by {@code IntakeSubsystem.<IntakeState>}
   */
  public void setState(IntakeState state) {
    targetState = state;
    setExtension(targetState.isDeployed);
    io.setVoltage(targetState.outputVolts * 12.0);

    Logger.recordOutput("Intake/SuckerTargetVolts", targetState.outputVolts);
    Logger.recordOutput("Intake/isDeployed", targetState.isDeployed);
  }

  /**
   * Sets the voltage of the intake motor
   *
   * @param volts the voltage requested
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Fires or retracts the pistons to extend the intake
   *
   * @param value true -> extended: false -> retracted
   */
  public void setExtension(boolean value) {
    io.setLinear(value);
  }

  /** A class with two finals variables that control the intake's state */
  public static class IntakeState {
    public final boolean isDeployed;
    public final double outputVolts;

    /**
     * Creates a new IntakeState with values input by the user
     *
     * @param outputVolts The output percent you want the intake to run at
     * @param angle A Rotation2d representing the tartget angle of the intake, with positive being
     *     forward away from the robot
     */
    public IntakeState(double outputVolts, boolean isDeployed) {
      this.isDeployed = isDeployed;
      this.outputVolts = outputVolts;
    }
  }
}
