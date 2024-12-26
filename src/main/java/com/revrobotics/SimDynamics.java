package com.revrobotics;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class SimDynamics {
  private DoubleSupplier m_getVelocityRPM;
  private DoubleSupplier m_getCurrentDrawAmps;
  private DoubleConsumer m_setInputVoltage;
  private DoubleConsumer m_update;
  private Function<Double, Boolean> m_limits;

  public static SimDynamics fromSim(FlywheelSim flywheel) {
    return new SimDynamics(
        () -> flywheel.getAngularVelocityRPM(),
        () -> flywheel.getCurrentDrawAmps(),
        (voltage) -> flywheel.setInputVoltage(voltage),
        (dt_seconds) -> flywheel.update(dt_seconds),
        (position) -> false);
  }

  public static SimDynamics fromSim(SingleJointedArmSim singleJointedArm) {
    return new SimDynamics(
        () -> Units.radiansPerSecondToRotationsPerMinute(singleJointedArm.getVelocityRadPerSec()),
        () -> singleJointedArm.getCurrentDrawAmps(),
        (voltage) -> singleJointedArm.setInputVoltage(voltage),
        (dt_seconds) -> singleJointedArm.update(dt_seconds),
        (position) -> false);
  }

  public SimDynamics(
      DoubleSupplier getVelocityRPM,
      DoubleSupplier getCurrentDrawAmps,
      DoubleConsumer setInputVoltage,
      DoubleConsumer update,
      Function<Double, Boolean> hitLimit) {
    m_getVelocityRPM = getVelocityRPM;
    m_getCurrentDrawAmps = getCurrentDrawAmps;
    m_setInputVoltage = setInputVoltage;
    m_update = update;
    m_limits = hitLimit;
  }

  public double getAngularVelocityRPM() {
    return m_getVelocityRPM.getAsDouble();
  }

  public void setInputVoltage(double vbus) {
    m_setInputVoltage.accept(vbus);
  }

  public void update(double dt_seconds) {
    m_update.accept(dt_seconds);
  }

  public double getCurrentDrawAmps() {
    return m_getCurrentDrawAmps.getAsDouble();
  }
}
