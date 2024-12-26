package frc.lib.telemetry;

public interface TelemetryNode {
  default void bind(TelemetryBuilder builder) {}
}
