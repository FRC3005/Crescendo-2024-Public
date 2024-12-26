package frc.lib.vendor.sensor;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.monitor.IsConnected;

public interface ColorSensor extends IsConnected {
  public Color getColor();
}
