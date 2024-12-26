package com.revrobotics;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkDutyCycleSensorSim {
  private final SimDouble m_velocity;
  private final SimDouble m_position;
  private final SimDouble m_absPosition;
  private final SimDouble m_frequency;
  private final SparkAbsoluteEncoder m_sensor;

  public SparkDutyCycleSensorSim(SparkAbsoluteEncoder encoder) {
    int id = encoder.spark.getDeviceId();
    SimDeviceSim sparkMaxSim = new SimDeviceSim("SPARK MAX" + " [" + id + "]");
    m_velocity = sparkMaxSim.getDouble("Absolute Sensor Velocity");
    m_position = sparkMaxSim.getDouble("Absolute Sensor Position");
    m_absPosition = sparkMaxSim.getDouble("Absolute Sensor Absolute Position");
    m_frequency = sparkMaxSim.getDouble("Absolute Sensor Frequency");
    m_sensor = encoder;
  }

  public double getVelocity() {
    return m_velocity.get();
  }

  public double getPosition() {
    return m_position.get();
  }

  public double getAbsolutePosition() {
    return m_absPosition.get();
  }

  public double getFrequency() {
    return m_frequency.get();
  }

  public void setVelocity(double velocity) {
    m_velocity.set(velocity);
  }

  public void setPosition(double position) {
    m_position.set(position);

    // TODO: Get inversion settings and position factor and also set abs position
  }

  public void setAbsolutePosition(double absPosition) {
    m_absPosition.set(absPosition);

    // TODO: Get inversion settings and position factor and also set position
  }

  public void setFrequency(double frequency) {
    m_frequency.set(frequency);
  }
}
