package frc.lib.odometry;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.geometry.Point2d;
import frc.robot.constants.Field;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.*;

public class CarpetTest {
  private static final double kEpsilon = 1E-9;

  Point2d origin = new Point2d(-Field.kBorderMargin, -Field.kBorderMargin);
  Point2d centerXY = new Point2d(Field.kWidth / 2.0, Field.kLength + Field.kBorderMargin);
  Carpet someCarpet =
      new Carpet(
          new CarpetZone(origin, centerXY, new Rotation2d(1.0)),
          new CarpetZone(
              Field.kWidth / 2.0, Field.kWidth, 0, Field.kLength / 2, new Rotation2d(2.0)),
          new CarpetZone(
              Field.kWidth / 2.0,
              Field.kWidth,
              Field.kLength / 2,
              Field.kLength,
              new Rotation2d()));

  @BeforeEach
  public void setup() {
    someCarpet.setFieldOrigin(Alliance.Red);
  }

  @Test
  public void carpet() {
    someCarpet.setFieldOrigin(Alliance.Red);
    Assertions.assertEquals(new Rotation2d(1.0), someCarpet.carpetDirection(1.0, 1.0));
    Assertions.assertEquals(new Rotation2d(1.0), someCarpet.carpetDirection(1.0, Field.kLength));
    Assertions.assertEquals(
        new Rotation2d(1.0), someCarpet.carpetDirection(Field.kWidth / 2.0 - kEpsilon, 1.0));
    Assertions.assertEquals(
        new Rotation2d(2.0), someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, 2.0));
    Assertions.assertEquals(
        new Rotation2d(2.0),
        someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, Field.kLength / 2.0 - kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(),
        someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, Field.kLength / 2.0 + kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(), someCarpet.carpetDirection(Field.kWidth, Field.kLength));

    someCarpet.setFieldOrigin(Alliance.Blue);
    Assertions.assertEquals(new Rotation2d(), someCarpet.carpetDirection(1.0, 1.0));
    Assertions.assertEquals(new Rotation2d(2.0), someCarpet.carpetDirection(1.0, Field.kLength));
    Assertions.assertEquals(
        new Rotation2d(), someCarpet.carpetDirection(Field.kWidth / 2.0 - kEpsilon, 1.0));
    Assertions.assertEquals(
        new Rotation2d(1.0), someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, 2.0));
    Assertions.assertEquals(
        new Rotation2d(1.0),
        someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, Field.kLength / 2.0 - kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(0.0),
        someCarpet.carpetDirection(Field.kWidth / 2.0 - kEpsilon, Field.kLength / 2.0 - kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(2.0),
        someCarpet.carpetDirection(Field.kWidth / 2.0 - kEpsilon, Field.kLength / 2.0 + kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(1.0),
        someCarpet.carpetDirection(Field.kWidth / 2.0 + kEpsilon, Field.kLength / 2.0 + kEpsilon));
    Assertions.assertEquals(
        new Rotation2d(1.0), someCarpet.carpetDirection(Field.kWidth, Field.kLength));
  }

  @Test
  public void carpetTransform() {
    List<Pair<Rotation2d, Double>> dataPoints =
        Arrays.asList(
            Pair.of(Rotation2d.fromDegrees(0), 0.5),
            Pair.of(Rotation2d.fromDegrees(175), 2.0),
            Pair.of(Rotation2d.fromDegrees(-175), 1.0),
            Pair.of(Rotation2d.fromDegrees(-90), 1.5),
            Pair.of(Rotation2d.fromDegrees(90), 1.0));
    CarpetTransform transform = new CarpetTransform(dataPoints, someCarpet);

    // Simple lookup on a point
    Assertions.assertEquals(2.0, transform.getTransform(Rotation2d.fromDegrees(175)));

    // Simple lookup between points
    Assertions.assertEquals(1.0, transform.getTransform(Rotation2d.fromDegrees(-45)));

    // Simple lookup between points that wrap at the end
    Assertions.assertEquals(1.5, transform.getTransform(Rotation2d.fromDegrees(180)));
    Assertions.assertEquals(1.5, transform.getTransform(Rotation2d.fromDegrees(-180)));
    Assertions.assertEquals(1.25, transform.getTransform(Rotation2d.fromDegrees(-177.5)));
    Assertions.assertEquals(1.75, transform.getTransform(Rotation2d.fromDegrees(177.5)));
  }

  @Test
  public void carpetTransformSimple() {
    CarpetTransform transform = new CarpetTransform(1.0, 2.0, someCarpet);
    Assertions.assertEquals(2.0, transform.getTransform(Rotation2d.fromDegrees(0)));
    Assertions.assertEquals(1.0, transform.getTransform(Rotation2d.fromDegrees(180)));
    Assertions.assertEquals(1.0, transform.getTransform(Rotation2d.fromDegrees(-180)));
    Assertions.assertEquals(1.5, transform.getTransform(Rotation2d.fromDegrees(90)));
    Assertions.assertEquals(1.5, transform.getTransform(Rotation2d.fromDegrees(-90)));
    Assertions.assertEquals(1.85, transform.getTransform(Rotation2d.fromDegrees(45)), 0.01);
    Assertions.assertEquals(1.85, transform.getTransform(Rotation2d.fromDegrees(-45)), 0.01);
  }
}
