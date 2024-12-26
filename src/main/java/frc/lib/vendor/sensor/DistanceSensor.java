package frc.lib.vendor.sensor;

import frc.lib.monitor.IsConnected;

public interface DistanceSensor extends IsConnected {
  public default double getDistanceInches() {
    return getDistanceMillimeters() * 25.4;
  }

  public double getDistanceMillimeters();
}
