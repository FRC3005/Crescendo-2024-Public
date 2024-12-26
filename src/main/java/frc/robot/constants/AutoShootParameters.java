package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.telemetry.TelemetryRunner;

public class AutoShootParameters {
  public static double kMaxShootDistanceMeters = 5.5;
  public static double kMaxAngularErrorRadians = Units.degreesToRadians(3.0);
  public static double kRpmTolerance = 250;
  public static double kMaxArmAngleErrorRadians = Units.degreesToRadians(0.6);

  public static double kMaxRotationalVelocityRadiansPerSecondOnTheMove = 100.0;

  public static double kMaxTransalationalVelocityMetersPerSecondOnTheMove = 3.5;
  public static double kMaxShootDistanceMetersOnTheMove = 4.5;
  public static double kMinShootDistanceMetersOnTheMove = 2.5;

  public static double kMaxTransalationalVelocityMetersPerSecond = 0.5;
  public static double kMaxRotationalVelocityRadiansPerSecond = 0.5;

  private static class AutoShootParams implements TelemetryNode {
    @Override
    public void bind(TelemetryBuilder builder) {
      if (!RobotConstants.kCompetitionMode) {
        builder.addDoubleProperty(
            "Dist Meters", () -> kMaxShootDistanceMeters, (val) -> kMaxShootDistanceMeters = val);
        builder.addDoubleProperty(
            "Angle Error rads",
            () -> kMaxAngularErrorRadians,
            (val) -> kMaxAngularErrorRadians = val);
        builder.addDoubleProperty("RPM Tol", () -> kRpmTolerance, (val) -> kRpmTolerance = val);
        builder.addDoubleProperty(
            "Arm angle rads",
            () -> kMaxArmAngleErrorRadians,
            (val) -> kMaxArmAngleErrorRadians = val);
        builder.addDoubleProperty(
            "Translation Vel",
            () -> kMaxTransalationalVelocityMetersPerSecond,
            (val) -> kMaxTransalationalVelocityMetersPerSecond = val);
        builder.addDoubleProperty(
            "Rotation Vel",
            () -> kMaxRotationalVelocityRadiansPerSecond,
            (val) -> kMaxRotationalVelocityRadiansPerSecond = val);
        builder.addDoubleProperty(
            "Distance on the move far",
            () -> kMaxShootDistanceMetersOnTheMove,
            (val) -> kMaxShootDistanceMetersOnTheMove = val);
        builder.addDoubleProperty(
            "Distance on the move near",
            () -> kMinShootDistanceMetersOnTheMove,
            (val) -> kMinShootDistanceMetersOnTheMove = val);
        builder.addDoubleProperty(
            "Distance velocity m/s on the move",
            () -> kMaxTransalationalVelocityMetersPerSecondOnTheMove,
            (val) -> kMaxTransalationalVelocityMetersPerSecondOnTheMove = val);
      }
    }
  }

  public static void bindTelemetry(TelemetryRunner runner) {
    runner.bind(new AutoShootParams());
  }
}
