package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) {
    // Small delay to give time between sets.
    if (Robot.isReal()) {
      Timer.delay(0.001);
    }
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static REVLibError setDefaultsForNeo(CANSparkBase spark) {
    // Send both even if one fails
    REVLibError error1 = spark.setSmartCurrentLimit(60);
    REVLibError error2 = spark.setSecondaryCurrentLimit(90);

    if (error1 != REVLibError.kOk) {
      return error1;
    }

    return error2;
  }

  public static REVLibError setDefaultsForNeo500(CANSparkBase spark) {
    // Send both even if one fails
    REVLibError error1 = spark.setSmartCurrentLimit(25);
    REVLibError error2 = spark.setSecondaryCurrentLimit(35);

    if (error1 != REVLibError.kOk) {
      return error1;
    }

    return error2;
  }

  public static class UnitConversions {
    public static REVLibError setDegreesFromGearRatio(
        RelativeEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    public static REVLibError setRadsFromGearRatio(RelativeEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }
  }

  public static String faultWordToString(short faults) {
    if (faults == 0) {
      return "";
    }

    StringBuilder builder = new StringBuilder();
    int faultsInt = faults;
    for (int i = 0; i < 16; i++) {
      if (((1 << i) & faultsInt) != 0) {
        builder.append(CANSparkBase.FaultID.fromId(i).toString());
        builder.append(" ");
      }
    }
    return builder.toString();
  }

  /*
   * This call takes extra time to set multiple times since there is no direct feedback from
   * the spark max on whether this succeeded or not.
   */
  public static REVLibError setPeriodicFramePeriod(
      CANSparkBase spark, PeriodicFrame frame, int periodMs) {
    REVLibError error = REVLibError.kOk;

    for (int i = 0; i < 3; i++) {
      Timer.delay(0.01);
      REVLibError e = spark.setPeriodicFramePeriod(frame, periodMs);
      if (e != REVLibError.kOk) {
        error = e;
      }
    }

    return error;
  }
}
