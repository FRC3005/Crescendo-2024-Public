package frc.lib.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Field;
import java.util.Arrays;
import java.util.List;

public class Carpet {
  public enum Brand {
    NEYLAND_II_66561_MEDALLION,
    NEYLAND_III_66561_MEDALLION
  };

  private final List<CarpetZone> m_carpetZones;
  private Alliance m_alliance = Alliance.Red;

  public Carpet(CarpetZone... zones) {
    m_carpetZones = Arrays.asList(zones);
  }

  public void setFieldOrigin(Alliance alliance) {
    m_alliance = alliance;
  }

  public Rotation2d carpetDirection(double x, double y) {
    // Rotate field when blue
    if (m_alliance == Alliance.Blue) {
      x = Field.kWidth - x;
      y = Field.kLength - y;
    }

    for (CarpetZone carpetZone : m_carpetZones) {
      if (carpetZone.containsPoint(x, y)) {
        return carpetZone.getCarpetDirection();
      }
    }

    return new Rotation2d();
  }

  public static Carpet centerLineCarpet(
      Rotation2d carpetDirectionOrigin, Rotation2d carpetDireciton) {
    return new Carpet(
        new CarpetZone(0, Field.kWidth / 2.0, 0, Field.kLength, carpetDirectionOrigin),
        new CarpetZone(Field.kWidth / 2.0, Field.kWidth, 0, Field.kLength, carpetDirectionOrigin));
  }
}
