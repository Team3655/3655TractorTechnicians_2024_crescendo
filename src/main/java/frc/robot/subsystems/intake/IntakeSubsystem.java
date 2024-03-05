// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public static final double SUCKER_GEAR_RATIO = 3.0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new intake. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    // tell the intake to tuck when the proxy sensor is tripped
    Trigger photoEye = new Trigger(() -> io.getProximity());
    photoEye.onTrue(IntakeCommands.retract(this));
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
   * Sets the voltage of the intake motor
   *
   * @param volts the voltage requested
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setStageOne(Value value) {
    io.setStageOneValue(value);
  }

  public void setStageTwo(Value value) {
    io.setStageTwoValue(value);
  }
}
