// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.vendor.motorcontroller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.tinylog.Logger;

/** This is a basic monitor class separate from the HealthMonitor setup. */
public class SparkMonitor extends SubsystemBase {
  private HashMap<SparkFlex, Short> m_sparkMaxs = new HashMap<>();
  private int m_runCount = 0;

  /** Creates a new SparkMaxMonitor. */
  public SparkMonitor() {}

  public boolean add(SparkFlex spark) {
    if (m_sparkMaxs.containsKey(spark)) {
      return false;
    }
    m_sparkMaxs.put(spark, (short) 0);
    return true;
  }

  @Override
  public void periodic() {
    // Run at 1 second
    if (m_runCount++ < 50) {
      return;
    }
    m_runCount = 0;

    m_sparkMaxs.forEach(
        (spark, prevFault) -> {
          short faults = spark.getStickyFaults();
          if (faults != prevFault.shortValue()) {
            Logger.tag("Spark Max Monitor")
                .warn(
                    "Spark Max ID {} faults: {}",
                    spark.getDeviceId(),
                    SparkMaxUtils.faultWordToString(faults));
          }
          m_sparkMaxs.put(spark, faults);
        });
  }
}
