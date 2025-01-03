// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import java.util.function.Function;

public class ProfiledPositionController implements TelemetryNode {
  private double m_dt = 0.02;
  private final Controller m_positionController;
  private AsymmetricTrapezoidProfile.Constraints m_profileConstraints;
  private AsymmetricTrapezoidProfile.State m_setpoint = new AsymmetricTrapezoidProfile.State();
  private AsymmetricTrapezoidProfile.State m_goal = new AsymmetricTrapezoidProfile.State();
  // TODO: This needs a refactor...
  private AsymmetricTrapezoidProfile.State m_initialSetpoint =
      new AsymmetricTrapezoidProfile.State();
  private double m_feedforward_volts = 0.0;
  private double m_minimumInput = 0.0;
  private double m_maximumInput = 0.0;

  public ProfiledPositionController(
      Controller positionController,
      AsymmetricTrapezoidProfile.Constraints constraints,
      double loopPeriod) {
    m_positionController = positionController;
    m_profileConstraints = constraints;
    m_dt = loopPeriod;
  }

  public ProfiledPositionController(
      Controller positionController, AsymmetricTrapezoidProfile.Constraints constraints) {
    m_positionController = positionController;
    m_profileConstraints = constraints;
  }

  public ProfiledPositionController(
      Controller positionController, TrapezoidProfile.Constraints constraints, double loopPeriod) {
    m_positionController = positionController;
    m_profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
    m_dt = loopPeriod;
  }

  public ProfiledPositionController(
      Controller positionController, TrapezoidProfile.Constraints constraints) {
    m_positionController = positionController;
    m_profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
  }

  public void setConstraints(AsymmetricTrapezoidProfile.Constraints constraints) {
    m_profileConstraints = constraints;
  }

  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
  }

  public AsymmetricTrapezoidProfile.State getGoal() {
    return m_goal;
  }

  public AsymmetricTrapezoidProfile.State getTarget() {
    return m_setpoint;
  }

  public AsymmetricTrapezoidProfile.State getInitialTarget() {
    return m_initialSetpoint;
  }

  /**
   * Get the direction of the profile computed by the controller. This is the output of the profile
   * not the output of the controller.
   *
   * @return -1 for negative output, 0 for no output
   */
  public double getProfileDirection() {
    return Math.signum(m_goal.position - m_setpoint.position);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindChild("Inner Position Loop", m_positionController);
    builder.addDoubleProperty("Position State", () -> m_setpoint.position, null);
    builder.addDoubleProperty("Velocity State", () -> m_setpoint.velocity, null);
    builder.addDoubleProperty("Position Goal", () -> m_goal.position, null);
    builder.addDoubleProperty("Velocity Goal", () -> m_goal.velocity, null);
    builder.addDoubleProperty("Feedforward Volts", () -> m_feedforward_volts, null);
    builder.addDoubleProperty("Max Velocity", () -> m_profileConstraints.maxVelocity, null);
    builder.addDoubleProperty("Max Acceleration", () -> m_profileConstraints.maxAcceleration, null);
    builder.addDoubleProperty("Max Deceleration", () -> m_profileConstraints.maxDeceleration, null);
    builder.addDoubleProperty("Profile Direction", this::getProfileDirection, null);
  }

  public void setReference(
      double reference,
      double measurement,
      Function<AsymmetricTrapezoidProfile.State, Double> feedforward) {
    if (reference != m_goal.position || m_resetState) {
      if (m_positionController.isContinuousInputEnabled()) {
        // Get error which is smallest distance between goal and measurement
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
        double goalMinDistance =
            MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
        double setpointMinDistance =
            MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

        // Recompute the profile goal with the smallest error, giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs
        // to be offset from the measurement by the input range modulus; they don't need to be
        // equal.
        m_goal.position = goalMinDistance + measurement;
        m_setpoint.position = setpointMinDistance + measurement;
        m_resetState = false;
      }
      m_goal = new AsymmetricTrapezoidProfile.State(reference, 0);
      m_initialSetpoint = m_setpoint;
    }

    var profile = new AsymmetricTrapezoidProfile(m_profileConstraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(m_dt);
    m_feedforward_volts = feedforward.apply(m_setpoint);
    m_positionController.setReference(m_setpoint.position, m_feedforward_volts);
  }

  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_positionController.enableContinuousInput(minimumInput, maximumInput);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  public void disableContinuousInput() {
    m_positionController.disableContinuousInput();
  }

  public boolean isContinuousInputEnabled() {
    return m_positionController.isContinuousInputEnabled();
  }

  public boolean atGoal() {
    return m_setpoint.equals(m_goal);
  }

  private boolean m_resetState = false;

  public void resetState() {
    m_resetState = true;
  }
}
