package frc.lib.electromechanical;

import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;

public interface Encoder extends TelemetryNode {
  public double getVelocity();

  public double getPosition();

  public void setPosition(double position);

  @Override
  default void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("Position", () -> this.getPosition(), null);
    builder.addDoubleProperty("Velocity", () -> this.getVelocity(), null);
  }
}
