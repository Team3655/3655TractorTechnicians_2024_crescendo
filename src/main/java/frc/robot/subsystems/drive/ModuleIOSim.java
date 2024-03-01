package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Implementation of the Swerve Module IO interface. Simulating the drive train of a robot through
 * software instead of hardware.
 */
public class ModuleIOSim implements ModuleIO {
  /** Wheel radius of a Swerve Module */
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  /** Simulated drive motor using the accurate gearing ratios and moment of inertia */
  private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), 6.12, 0.025);
  /** Simulated steer motor using the accurate gearing ratios and moment of inertia */
  private final DCMotorSim steerSim =
      new DCMotorSim(DCMotor.getKrakenX60(1), 150.0 / 7.0, 0.004096955);
  /** PID Controller for drive motor. Will do our target drive velocity to voltage calculation */
  private final PIDController driveFeedback = new PIDController(10, 0, 0);
  /** PID Controller for steer motor. Will do our target angular position to voltage calculation */
  private final PIDController steerFeedback = new PIDController(10, 0, 0);
  /** Represents the swerve module encoder's absolute value */
  private double steerRelativePositionRad = 0;
  /** Represents a starting angular position value for our MOTOR encoder */
  private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

  private double currentTime = 0.0;
  private double lastTime = 0.0;

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {

    lastTime = currentTime;
    currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastTime;

    // Calculating target data to voltage data
    double driveAppliedVolts = driveFeedback.calculate(inputs.driveVelocityMetersPerSec);
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    double steerAppliedVolts = steerFeedback.calculate(inputs.steerPositionRad);
    steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);

    // Applying calculated voltage data to simulated motors
    driveSim.setInputVoltage(driveAppliedVolts);
    steerSim.setInputVoltage(steerAppliedVolts);

    // Updates every set amount of seconds (replicating a robot, we are using the default time
    // between updates for
    // the robot)
    driveSim.update(deltaTime);
    steerSim.update(deltaTime);

    // Adding the difference in angles between the absolute and relative angles to the current angle
    // (correcting for
    // the offset)
    double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * deltaTime;
    steerRelativePositionRad += angleDiffRad;
    steerAbsolutePositionRad += angleDiffRad;

    // Limiting the domain of the absolute angular position to be in between 0 and 2π
    steerAbsolutePositionRad = MathUtil.inputModulus(steerAbsolutePositionRad, 0, 2 * Math.PI);

    // Setting the inputs within the IO class
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * WHEEL_RADIUS_METERS;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.steerPositionRad = steerRelativePositionRad;
    inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.steerCurrentDrawAmps = Math.abs(steerSim.getCurrentDrawAmps());

    inputs.steerAbsolutePosition = steerAbsolutePositionRad;

    inputs.targetDriveVelocityMetersPerSec = driveFeedback.getSetpoint();
    inputs.targetSteerPositionRad = steerFeedback.getSetpoint();
  }

  @Override
  public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
    driveFeedback.setSetpoint(
        targetDriveVelocityMetersPerSec); // Setting the target point for the PID controller to
    // calculate the
    // voltage to
  }

  @Override
  public void setTargetSteerPosition(double targetSteerPositionRad) {
    steerFeedback.setSetpoint(
        targetSteerPositionRad); // Setting the target point for the PID controller to calculate the
    // voltage to
  }
}
