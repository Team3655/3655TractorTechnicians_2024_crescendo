// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pneumatics extends SubsystemBase {

  public static final int PNEUMATIC_HUB_ID = 50;
  public static final int MAX_PRESSURE_PSI = 120;
  public static final int MIN_PRESSURE_PSI = 100;

  private final PneumaticIO io;
  private final PneumaticIOInputsAutoLogged inputs = new PneumaticIOInputsAutoLogged();

  /** Creates a new Pneumatics. */
  public Pneumatics(PneumaticIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pneumatics", inputs);
  }
}
