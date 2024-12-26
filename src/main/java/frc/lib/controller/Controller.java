package frc.lib.controller;

import frc.lib.telemetry.TelemetryNode;

public interface Controller extends TelemetryNode {

  /**
   * Set the controller target reference and run it
   *
   * @param setpoint is the controller setpoint
   */
  default void setReference(double setpoint) {
    setReference(setpoint, 0.0);
  }

  /**
   * Set the controller target reference and run it
   *
   * @param setpoint controller setpoint
   * @param feedforward feedforward value
   */
  void setReference(double setpoint, double feedforward);

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  void enableContinuousInput(double minimumInput, double maximumInput);

  /** Disable continuous input */
  void disableContinuousInput();

  /**
   * Returns true if continuous input is enabled.
   *
   * @return True if continuous input is enabled.
   */
  public boolean isContinuousInputEnabled();
}
