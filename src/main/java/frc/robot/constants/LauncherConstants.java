package frc.robot.constants;

import frc.lib.controller.PIDGains;

public final class LauncherConstants {
  public static final PIDGains kPidGains = new PIDGains(0.005, 0.0, 0.0);
  public static final double kGearRatio = 1.0;
  public static final double kMOI = (0.00042187) + (0.00003973);
  // Use 'magic' number so setpoint is never equal
  public static final double kIdleRpm = 1000.123456;
  public static final double kS = 0.2;
  public static final double kV = 0.0017;

  public static final double kShooterRatio = 1.46985;
  public static final double kLowPassRpm = 2200.0;
}
