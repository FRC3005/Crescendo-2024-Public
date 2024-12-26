package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tinylog.Logger;

public class Game {
  private static Translation2d cachedTranslation = new Translation2d();
  private static int lastTagId = 0;

  private static final double ShuttleLocationYMeters = 4.5;
  private static final double ShuttleLocationYIsolateMeters = 6.5;

  private static final String kSmartDashboardName = "Pass To Isolated Station";

  // TODO: This should go elsewhere
  public static boolean shootToIsolated() {
    return SmartDashboard.getBoolean(kSmartDashboardName, false);
  }

  public static void registerSmartDashboardOnInit() {
    SmartDashboard.putBoolean(kSmartDashboardName, false);
  }

  private static final Translation2d kBlueAimTranslation = new Translation2d(0.217, 5.548);
  private static final Translation2d kRedAimTranslation = new Translation2d(16.324, 5.548);

  private static final Translation2d kShuttleLocationBlue =
      new Translation2d(1.13, ShuttleLocationYMeters);
  private static final Translation2d kShuttleLocationRed =
      new Translation2d(15.373, ShuttleLocationYMeters);

  private static final Translation2d kShuttleLocationIsolatedBlue =
      new Translation2d(1.13, ShuttleLocationYIsolateMeters);
  private static final Translation2d kShuttleLocationIsolatedRed =
      new Translation2d(15.373, ShuttleLocationYIsolateMeters);

  // Get the tag for your alliance that is the speaker center tag
  public static int getSpeakerCenterTagId() {
    return Robot.isRedAlliance() ? 4 : 7;
  }

  private static void checkAndUpdateAlliance() {
    int tagId = getSpeakerCenterTagId();
    if (tagId != lastTagId) {
      lastTagId = tagId;
      cachedTranslation = tagId == 4 ? kRedAimTranslation : kBlueAimTranslation;
      Logger.tag("Game")
          .info(
              "Setting speaker tag target to {}, speaker translation: {}",
              tagId,
              cachedTranslation);
    }
  }

  public static Translation2d getSpeakerShotTranslation() {
    checkAndUpdateAlliance();
    return cachedTranslation;
  }

  /** This is the location in front of the goal */
  public static Translation2d getLongDistanceShuttleLocation() {
    boolean targetIsolated = SmartDashboard.getBoolean(kSmartDashboardName, false);
    if (targetIsolated) {
      return Robot.isRedAlliance() ? kShuttleLocationIsolatedRed : kShuttleLocationIsolatedBlue;
    } else {
      return Robot.isRedAlliance() ? kShuttleLocationRed : kShuttleLocationBlue;
    }
  }
}
