// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {

  private final DoubleSolenoidSim sim = new DoubleSolenoidSim(0, PneumaticsModuleType.REVPH, 0, 0);
  

}
