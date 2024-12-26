package frc.lib.odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.ArrayList;
import java.util.List;

public class CarpetTransform {
  /** Key = theta (radians), Value = percent of effective wheel radius change (1.0 == 100%) */
  private final InterpolatingDoubleTreeMap m_lookup = new InterpolatingDoubleTreeMap();

  private final Carpet m_carpet;

  /**
   * Specify carpet transform using just the min and max measured values. To calculate the easy way.
   *
   * <p>1) Specify a 'nominal' wheel diameter. This will go in the DrivetrainConstants.java file. An
   * easy way to find this si to run the wheel diameter characterization.
   *
   * <p>2) Measure the effective wheel diameter 'with' and 'against' the grain. This should be the
   * min and max measured across all angles of the carpet.
   *
   * <p>3) minScalar is minDiameter / nominalDiameter maxScalar is maxDiameter / nominalDiameter
   *
   * @param minScalar minDiameter / nominalDiameter
   * @param maxScalar maxDiameter / nominalDiameter
   * @param carpet defines the carpet to drive on
   */
  public CarpetTransform(double minScalar, double maxScalar, Carpet carpet) {
    List<Pair<Rotation2d, Double>> points = new ArrayList<>();

    double amplitude = (maxScalar - minScalar) / 2.0;
    double offset = (maxScalar + minScalar) / 2.0;
    // For simplicity create a lookup using cos function, can be fairly sparse
    for (int i = -180; i <= 180; i += 30) {
      Rotation2d rotation = Rotation2d.fromDegrees(i);
      points.add(new Pair<Rotation2d, Double>(rotation, rotation.getCos() * amplitude + offset));
    }
    m_carpet = carpet;
    initialize(points);
  }

  public CarpetTransform(List<Pair<Rotation2d, Double>> points, Carpet carpet) {
    m_carpet = carpet;
    initialize(points);
  }

  private final void initialize(List<Pair<Rotation2d, Double>> points) {
    double min = MathUtil.angleModulus(points.get(0).getFirst().getRadians());
    double max = min;
    for (var pair : points) {
      double angle = MathUtil.angleModulus(pair.getFirst().getRadians());

      if (angle < min) {
        min = angle;
      } else if (angle > max) {
        max = angle;
      }

      m_lookup.put(angle, pair.getSecond());
    }

    // Make sure the interpolation wraps
    if (min == -Math.PI && max == Math.PI) {
      // TODO: Should this throw an error if the two points are too far from eachother?
      return;
    }

    double minVal = m_lookup.get(min);
    double maxVal = m_lookup.get(max);
    double maxDelta = (Math.PI - max);
    double minDelta = (Math.PI - Math.abs(min));
    double midpoint = minDelta / (minDelta + maxDelta);
    double wrapValue = MathUtil.interpolate(minVal, maxVal, midpoint);

    if (min > -Math.PI) {
      m_lookup.put(-Math.PI, wrapValue);
    }
    if (max < Math.PI) {
      m_lookup.put(Math.PI, wrapValue);
    }
  }

  public double getTransform(Rotation2d angle) {
    double angleRads = MathUtil.angleModulus(angle.getRadians());
    return m_lookup.get(angleRads);
  }

  public double getTransform(Pose2d pose) {
    Rotation2d carpetDirection = m_carpet.carpetDirection(pose.getX(), pose.getY());
    Rotation2d correctedDirection = pose.getRotation().minus(carpetDirection);
    return m_lookup.get(MathUtil.angleModulus(correctedDirection.getRadians()));
  }
}
