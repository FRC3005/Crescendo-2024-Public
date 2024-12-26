package frc.lib.electromechanical;

import frc.lib.monitor.IsConnected;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;

public interface AbsoluteEncoder extends TelemetryNode, IsConnected {
  public double getPosition();

  @Override
  default void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("Position", () -> this.getPosition(), null);
  }
}
