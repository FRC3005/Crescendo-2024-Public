package frc.lib.vendor.sensor;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.telemetry.TelemetryNode;

public interface SendableGyro extends Gyro, TelemetryNode {
  public void setAngle(double angleDegreesCCWPositive);
}
