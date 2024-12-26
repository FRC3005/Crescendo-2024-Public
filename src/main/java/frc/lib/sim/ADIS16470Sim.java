// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.lib.vendor.sensor.ADIS16470;

/** Class to control a simulated ADIS16470 gyroscope. */
@SuppressWarnings({"TypeName", "AbbreviationAsWordInName"})
public class ADIS16470Sim {
  private final SimDouble m_simGyroAngleX;

  /**
   * Constructs from an ADIS16470_IMU object.
   *
   * @param gyro ADIS16470_IMU to simulate
   */
  public ADIS16470Sim(ADIS16470 gyro) {
    SimDeviceSim wrappedSimDevice = new SimDeviceSim("Gyro:ADIS16470" + "[" + gyro.getPort() + "]");
    m_simGyroAngleX = wrappedSimDevice.getDouble("angle_x");
  }

  /**
   * Sets the X axis angle in degrees (CCW positive).
   *
   * @param angleDegrees The angle.
   */
  public void setGyroAngleX(double angleDegrees) {
    m_simGyroAngleX.set(angleDegrees);
  }
}
