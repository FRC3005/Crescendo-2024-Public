package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.geometry.Point2d;
import frc.lib.odometry.Carpet;
import frc.lib.odometry.CarpetZone;

public final class Field {
  public static final double kLength = Units.inchesToMeters(651.25);
  public static final double kMaxX = kLength;
  public static final double kWidth = Units.inchesToMeters(323.25);
  public static final double kMaxY = kWidth;
  public static final double kBorderMargin = 0.5;

  public static final class HomeField {
    public static final double kMidLineY = kWidth / 2;
    public static final double kMidLineX = Field.kLength / 2.0 + 1.905;
  }

  /**
   * Start at the red amp (standing on the red side, looking at the blue side, all the way right).
   * Look at the carpet. If the nap goes down towards blue, the angle is 0 for the first number. (If
   * you lightly push the carpet forward towards blue, it has less resistance compared to pulling
   * back towards red). Otherwise if its laying towards red, this number is 180 degrees. Range
   * should be [-180, 180] (CCW+) The second number is the same experiment, but standing on the
   * feeder station (standing on the red side, looking at the blue side, all the way left).
   */
  public static final Carpet kFtWorth =
      Carpet.centerLineCarpet(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));

  public static final Carpet kHomeCarpet =
      new Carpet(
          // Red side loading
          new CarpetZone(
              new Point2d(-1, -1),
              new Point2d(HomeField.kMidLineX, HomeField.kMidLineY),
              Rotation2d.fromDegrees(180)),
          // Red side wall
          new CarpetZone(
              new Point2d(-1, HomeField.kMidLineY),
              new Point2d(HomeField.kMidLineX, 100),
              new Rotation2d()),
          // Blue side loading
          new CarpetZone(
              new Point2d(HomeField.kMidLineX, -1),
              new Point2d(100, HomeField.kMidLineY),
              Rotation2d.fromDegrees(180)),
          // Blue side wall
          new CarpetZone(
              new Point2d(HomeField.kMidLineX, HomeField.kMidLineY),
              new Point2d(100, 100),
              Rotation2d.fromDegrees(180)));

  // Finally, set the actual competition carpet
  public static final Carpet kCompetitionCarpet = kFtWorth;

  static {
    // This year blue is always the origin
    kCompetitionCarpet.setFieldOrigin(Alliance.Blue);
  }
}
