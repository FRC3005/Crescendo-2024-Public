package com.revrobotics;

public class CANSparkBaseHandle {
  public final long handle;

  public CANSparkBaseHandle(CANSparkBase spark) {
    handle = spark.sparkMaxHandle;
  }
}
