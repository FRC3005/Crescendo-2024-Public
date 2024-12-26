package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import frc.robot.constants.LauncherConstants;

public class LauncherSetpoint {
  public final double leftRpm;
  public final double rightRpm;

  public LauncherSetpoint(double leftRpm, double rightRpm) {
    this.leftRpm = leftRpm;
    this.rightRpm = rightRpm;
  }

  public LauncherSetpoint(double leftRpm) {
    this.leftRpm = leftRpm;
    this.rightRpm = leftRpm / LauncherConstants.kShooterRatio;
  }

  private static LauncherSetpoint interpolate(
      LauncherSetpoint startValue, LauncherSetpoint endValue, double t) {
    return new LauncherSetpoint(
        MathUtil.interpolate(startValue.leftRpm, endValue.leftRpm, t),
        MathUtil.interpolate(startValue.rightRpm, endValue.rightRpm, t));
  }

  public static Interpolator<LauncherSetpoint> interpolator() {
    return LauncherSetpoint::interpolate;
  }
}
