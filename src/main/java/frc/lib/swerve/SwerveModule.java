// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.telemetry.TelemetryNode;

public interface SwerveModule extends TelemetryNode {
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /**
   * Set the module to keep its current heading, with a certain speed. Useful for holding rotation
   * e.g. when the drive sticks are not pressed.
   *
   * @param speedMetersPerSecond the speed to set the module to.
   */
  public void holdHeading(double speedMetersPerSecond);

  /** Reset the drive encoder to zero. */
  public void resetEncoders();

  /**
   * Get the module position. This is the same as getState, except with the position of the wheel
   * instead of velocity.
   *
   * @return Current module position.
   */
  public SwerveModulePosition getPosition();

  /**
   * Periodic funcion runs at the rate of the swerve drive. Used in the case that a PID controller
   * must be run continuously or similar.
   */
  public default void periodic() {}

  public void setVoltageAngle(double voltage, Rotation2d angle);

  public default void simulationPeriodic() {}

  public default void testPeriodic() {}

  public SwerveModuleState getActualState();

  /**
   * Run a specific module setpoint. This includes (robot centric) turing theta in radians, and
   * speed in meters per second. Use this to run directly to a state without any call to optimize.
   * Used when running 254 setpoint planner.
   *
   * @param state the state to set the module to.
   */
  public void runSetpoint(SwerveModuleState state);

  public void setDriveCurrent(int currentAmps);
}
