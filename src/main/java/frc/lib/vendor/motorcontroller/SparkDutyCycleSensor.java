package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.lib.electromechanical.AbsoluteEncoder;

public class SparkDutyCycleSensor implements AbsoluteEncoder {
  private final SparkAbsoluteEncoder m_sensor;

  public SparkDutyCycleSensor(CANSparkBase spark) {
    m_sensor = spark.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public double getPosition() {
    return m_sensor.getPosition();
  }

  public void setPositionOffset(double position) {
    m_sensor.setZeroOffset(position);
  }

  public SparkAbsoluteEncoder getSensor() {
    return m_sensor;
  }
}
