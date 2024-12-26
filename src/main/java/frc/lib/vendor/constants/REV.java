package frc.lib.vendor.constants;

public final class REV {
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676.0;
    public static final double kFreeSpeedRadPerSec = kFreeSpeedRpm / 60.0 * 2.0 * Math.PI;
  }

  public static final class Neo550MotorConstants {
    public static final double kFreeSpeedRpm = 11000.0;
    public static final double kFreeSpeedRadPerSec = kFreeSpeedRpm / 60.0 * 2.0 * Math.PI;
  }

  public static final class UltraPlanetaryConstants {
    public static final double kThreeToOneRatio = 29.0 / 84.0;
    public static final double kFourToOneRatio = 21.0 / 76.0;
    public static final double kFiveToOneRatio = 13.0 / 68.0;
  }
}
