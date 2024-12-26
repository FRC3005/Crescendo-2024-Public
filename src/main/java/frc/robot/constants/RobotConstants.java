package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class RobotConstants {
  public static final double kMassLbs = 80.0;
  public static final double kMassKg = Units.lbsToKilograms(kMassLbs);

  /**
   * Whether in competition mode or not. Competition mode has less telemetry (specific to tuning
   * controls), less error message, and less verbose logging. It may also have small changes to
   * calibration times e.g. gyro. Manually set this value before events.
   */
  public static final boolean kCompetitionMode = false;

  public static final boolean kReducedTelemetryMode = false;
}
