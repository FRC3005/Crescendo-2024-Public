package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

public class CANSparkBaseExtensions {
  /**
   * Checked function for setting controller inverted.
   *
   * <p>This call has no effect if the controller is a follower. To invert a follower, see the
   * follow() method.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public static REVLibError setInverted(CANSparkBase spark, boolean isInverted) {
    spark.throwIfClosed();
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetInverted(spark.sparkMaxHandle, isInverted));
  }

  /** Enable center aligned mode for the duty cycle sensor. */
  public static REVLibError enableCenterAlignedMode(CANSparkBase spark) {
    CANSparkBaseHandle handle = new CANSparkBaseHandle(spark);
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(handle.handle, 152, true));
  }

  /** Disable center aligned mode for the duty cycle sensor. */
  public static REVLibError disableCenterAlignedMode(CANSparkBase spark) {
    CANSparkBaseHandle handle = new CANSparkBaseHandle(spark);
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(handle.handle, 152, false));
  }

  /**
   * Enable mode which sets the output of the PID controllers to be voltage instead of duty cycle.
   *
   * <p>To disable, change or disable voltage compensation. Those settings will overwrite this one
   */
  public static REVLibError enablePIDVoltageOutput(CANSparkBase spark) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterUint32(spark.sparkMaxHandle, 74, 1));
  }

  /**
   * Disable mode which sets the output of the PID controllers to be voltage instead of duty cycle.
   *
   * <p>This is disable all voltage compensation. Normal voltage compensation will override this
   */
  public static REVLibError disablePIDVoltageOutput(CANSparkBase spark) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterUint32(spark.sparkMaxHandle, 74, 0));
  }

  /**
   * Workaround for issue where SPARK Flex API doesn't allow you to set this.
   *
   * @param spark CANSparkFlex to set
   * @param depth Encoder Average depth in the range [1, 64] default is 64
   * @return REVLibError::kOk if all is good
   */
  public static REVLibError setFlexEncoderAverageDepth(CANSparkFlex spark, int depth) {
    if (depth < 1 || depth > 64) {
      throw new IllegalArgumentException(
          "Quadrature average depth must be in the range of [1, 64]");
    }
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetAverageDepth(spark.sparkMaxHandle, depth));
  }

  /**
   * Workaround for issue where SPARK Flex API doesn't allow you to set this.
   *
   * @param spark CANSparkFlex to set
   * @param period_ms the sample delta period is milliseconds (derivative delta) default is 100
   * @return REVLibError::kOk if all is good
   */
  public static REVLibError setFlexEncoderSampleDelta(CANSparkFlex spark, int period_ms) {
    if (period_ms < 1 || period_ms > 100) {
      throw new IllegalArgumentException(
          "Quadrature measurement period must be in the range of [1, 100]");
    }
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetMeasurementPeriod(spark.sparkMaxHandle, period_ms));
  }

  /**
   * Set only the stall limit. This could be useful to only send 1 parameter instead of 3 if the
   * free speed RPM is not changed
   *
   * @param spark Spark to set
   * @param currentLimitAmps amps to set the current limit
   * @return Error if the call fails
   */
  public static REVLibError setSmartCurrentLimitStallOnly(
      CANSparkFlex spark, int currentLimitAmps) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterUint32(spark.sparkMaxHandle, 59, currentLimitAmps));
  }

  /**
   * Set only the secondary limit. This could be useful to only send 1 parameter instead of 2 if the
   * chop cycles is not changed.
   *
   * @param spark Spark to set
   * @param currentLimitAmps amps to set the current limit
   * @return Error if the call fails
   */
  public static REVLibError setSecondaryCurrentLimit(CANSparkFlex spark, int currentLimitAmps) {
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterFloat32(
            spark.sparkMaxHandle, 11, (float) currentLimitAmps));
  }
}
