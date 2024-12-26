package frc.lib.controller;

import edu.wpi.first.math.controller.PIDController;

public class PIDControllerWrapper implements Controller {
  private final PIDController m_controller;

  public PIDControllerWrapper(PIDController controller) {
    m_controller = controller;
  }

  @Override
  public void setReference(double setpoint, double feedforward) {
    m_controller.setSetpoint(setpoint + feedforward);
  }

  @Override
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_controller.enableContinuousInput(minimumInput, maximumInput);
  }

  @Override
  public void disableContinuousInput() {
    m_controller.disableContinuousInput();
  }

  @Override
  public boolean isContinuousInputEnabled() {
    return m_controller.isContinuousInputEnabled();
  }
}
