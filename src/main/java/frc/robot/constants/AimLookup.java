package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.launcher.LauncherSetpoint;

public class AimLookup {
  private static final InterpolatingDoubleTreeMap m_armAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_flightTimeLookup =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingTreeMap<Double, LauncherSetpoint> m_shooterLookup =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherSetpoint.interpolator());

  private static final InterpolatingTreeMap<Double, LauncherSetpoint> m_shuttleLookup =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherSetpoint.interpolator());

  private static final InterpolatingDoubleTreeMap m_shuttleArmAngle =
      new InterpolatingDoubleTreeMap();

  private static final double kShooterRatio = LauncherConstants.kShooterRatio;

  private static void entry(
      double distance, double armAngleDeg, double rightRpm, double flightTime) {
    m_armAngle.put(distance, Units.degreesToRadians(armAngleDeg));
    m_shooterLookup.put(distance, new LauncherSetpoint(rightRpm / kShooterRatio, rightRpm));
    m_flightTimeLookup.put(distance, flightTime);
  }

  private static void shuttleEntry(double distance, double armAngle, double rpm) {
    m_shuttleLookup.put(distance, new LauncherSetpoint(rpm, rpm / kShooterRatio));
    m_shuttleArmAngle.put(distance, Units.degreesToRadians(armAngle));
  }

  public static void initialize() {
    entry(1.025, 58.0, 3143.0, 0.133);
    entry(1.315, 53.0, 3143.0, 0.142);
    entry(1.720, 46.5, 3143.0, 0.174);
    entry(2.259, 40.0, 3286.0, 0.197);
    entry(2.758, 35.2, 3571.0, 0.206);
    entry(3.295, 31.7, 3786.0, 0.206);
    entry(3.810, 28.9, 4000.0, 0.242);
    entry(4.985, 24.9, 4286.0, 0.289);
    entry(5.914, 22.3, 4714.0, 0.289);
    entry(6.836, 21.8, 5143.0, 0.393);

    // These are red setpoints, when on blue, rpms are switched
    shuttleEntry(4.0, 62.0, 1900);
    shuttleEntry(5.0, 56.0, 2100);
    shuttleEntry(7.0, 49.0, 2500);
    shuttleEntry(9.0, 41.0, 2800);
    shuttleEntry(10.5, 38.0, 3200);
    shuttleEntry(12.0, 34.0, 3600);
    shuttleEntry(15.0, 33.0, 4200);
  }

  private static final double kFudgeFactor = Units.degreesToRadians(0.0);

  public static double armAngle(double distance) {
    return m_armAngle.get(distance) + kFudgeFactor;
  }

  public static LauncherSetpoint launcherSetpoint(double distance) {
    return m_shooterLookup.get(distance);
  }

  public static double flightTime(double distance) {
    return m_flightTimeLookup.get(distance);
  }

  public static LauncherSetpoint shuttleSetpoint(double distance) {
    LauncherSetpoint setpoint = m_shuttleLookup.get(distance);
    if (Robot.isRedAlliance()) {
      return setpoint;
    } else {
      return new LauncherSetpoint(setpoint.rightRpm, setpoint.leftRpm);
    }
  }

  public static double shuttleArmAngle(double distance) {
    return m_shuttleArmAngle.get(distance);
  }
}
