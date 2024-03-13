// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public static final double INTAKE_GEAR_RATIO = 3.0;
  public static final double FEEDER_GEAR_RATIO = 5.0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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

  /**
   * Sets the voltage of the intake motor
   *
   * @param volts the voltage requested
   */
  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  /**
   * Sets the voltage of the feeder motor
   *
   * @param volts the voltage requested
   */
  public void setFeederVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setStageOne(Value value) {
    io.setStageOneValue(value);
  }

  public void setStageTwo(Value value) {
    io.setStageTwoValue(value);
  }
}
