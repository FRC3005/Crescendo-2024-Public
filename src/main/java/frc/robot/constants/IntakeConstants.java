package frc.robot.constants;

import frc.lib.controller.PIDGains;

public final class IntakeConstants {
  public static final PIDGains kPositionGains = new PIDGains(0.1, 0.0, 0.0);

  public static final int kSmartCurrentLimit = 40;
  public static final int kSecondaryCurrentLimit = 45;

  public static final double kFeedTimeToShooterSeconds = 0.1;
}
