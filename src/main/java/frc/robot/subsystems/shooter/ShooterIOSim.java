package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private FlywheelSim flywheelSim1 = new FlywheelSim(DCMotor.getNeoVortex(1), 1, 0.004);
  private FlywheelSim flywheelSim2 = new FlywheelSim(DCMotor.getNeoVortex(1), 1, 0.004);

  private DCMotorSim pivotSim =
      new DCMotorSim(DCMotor.getNEO(1), ShooterSubsystem.PIVOT_GEAR_RATIO, 0.125);

  private PIDController flywheelPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController pivotPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double flywheelFFVolts = 0.0;
  private double flywheelVolts = 0.0;
  private double pivotFFVolts = 0.0;
  private double pivotVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    // the shooter needs to be able to run in open loop at zero volts
    // this is how the shooter coasts to a stop and is more gentle on the motors
    if (closedLoop) {
      flywheelVolts =
          MathUtil.clamp(
              flywheelPID.calculate(flywheelSim1.getAngularVelocityRadPerSec()) + flywheelFFVolts,
              -12.0,
              12.0);
    }

    pivotVolts =
        MathUtil.clamp(
            pivotPID.calculate(pivotSim.getAngularPositionRad()) + pivotFFVolts, -12.0, 12.0);

    flywheelSim1.setInputVoltage(flywheelVolts);
    flywheelSim2.setInputVoltage(flywheelVolts);
    pivotSim.setInputVoltage(pivotVolts);

    // progress the simulation by 1/50 of a second
    flywheelSim1.update(0.02);
    flywheelSim2.update(0.02);
    pivotSim.update(0.02);

    // top flywheel inputs
    inputs.topPositionRad = 0.0;
    inputs.topVelocityRPM = flywheelSim1.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = flywheelVolts;
    inputs.topCurrentAmps = new double[] {flywheelSim1.getCurrentDrawAmps()};

    // bottom flywheel inputs
    inputs.bottomPositionRad = 0.0;
    inputs.bottomVelocityRPM = flywheelSim2.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = flywheelVolts;
    inputs.bottomCurrentAmps = new double[] {flywheelSim2.getCurrentDrawAmps()};

    // pivot inputs
    inputs.pivotAbsolutePosition = Rotation2d.fromRadians(pivotSim.getAngularPositionRad());
    inputs.pivotPositionRotations = Units.radiansToRotations(pivotSim.getAngularPositionRad());
    inputs.pivotVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(pivotSim.getAngularVelocityRadPerSec());
    inputs.pivotAppliedVolts = pivotVolts;
    inputs.pivotCurrentAmps = new double[] {pivotSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    flywheelVolts = 0.0;
    flywheelSim1.setInputVoltage(volts);
    flywheelSim2.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    flywheelPID.setSetpoint(velocityRadPerSec);
    this.flywheelFFVolts = ffVolts;
  }

  @Override
  public void setAngle(Rotation2d angle, double ffVolts) {
    flywheelPID.setSetpoint(angle.getRotations());
    flywheelFFVolts = ffVolts;
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
