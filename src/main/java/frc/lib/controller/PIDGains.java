package frc.lib.controller;

import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;

public class PIDGains implements TelemetryNode {
  public PIDGains(double p, double i, double d) {
    P = p;
    I = i;
    D = d;
  }

  public PIDGains() {}

  public double P = 0.0;
  public double I = 0.0;
  public double D = 0.0;

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("P", () -> P, (val) -> P = val);
    builder.addDoubleProperty("I", () -> I, (val) -> I = val);
    builder.addDoubleProperty("D", () -> D, (val) -> D = val);
  }
}
