package frc.lib.vendor.sensor;

import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;

public interface ColorDistanceSensor extends TelemetryNode, DistanceSensor, ColorSensor {

  @Override
  public default void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("R", () -> getColor().red, null);
    builder.addDoubleProperty("G", () -> getColor().green, null);
    builder.addDoubleProperty("B", () -> getColor().blue, null);
    builder.addDoubleProperty("Distance mm", () -> getDistanceMillimeters(), null);
  }

  @Override
  default boolean isConnected() {
    return DistanceSensor.super.isConnected() && ColorSensor.super.isConnected();
  }
}
