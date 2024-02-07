package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private FlywheelSim flywheelSim1 = new FlywheelSim(DCMotor.getNeoVortex(1), 1, 0.004);
  private FlywheelSim flywheelSim2 = new FlywheelSim(DCMotor.getNeoVortex(1), 1, 0.004);

  private DCMotorSim pivotSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0);

  private PIDController flywheelPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController pivotPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    appliedVolts =
        MathUtil.clamp(
            flywheelPID.calculate(flywheelSim1.getAngularVelocityRadPerSec()) + ffVolts,
            -12.0,
            12.0);

    flywheelSim1.setInputVoltage(appliedVolts);
    flywheelSim2.setInputVoltage(appliedVolts);

    flywheelSim1.update(0.02);
    flywheelSim2.update(0.02);

    // top flywheel inputs
    inputs.topPositionRad = 0.0;
    inputs.topVelocityRPM = flywheelSim1.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = appliedVolts;
    inputs.topCurrentAmps = new double[] {flywheelSim1.getCurrentDrawAmps()};

    // bottom flywheel inputs
    inputs.bottomPositionRad = 0.0;
    inputs.bottomVelocityRPM = flywheelSim2.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = appliedVolts;
    inputs.bottomCurrentAmps = new double[] {flywheelSim2.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    flywheelSim1.setInputVoltage(volts);
    flywheelSim2.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    flywheelPID.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stopFlywheel() {
    setVoltage(0.0);
  }

  @Override
  public void configureFlywheelPID(double kP, double kI, double kD) {
    flywheelPID.setPID(kP, kI, kD);
  }

  @Override
  public void configurePivotPID(double kP, double kI, double kD) {
    pivotPID.setPID(kP, kI, kD);
  }
}
