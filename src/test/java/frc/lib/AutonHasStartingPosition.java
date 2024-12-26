package frc.lib;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.junit.jupiter.api.*;

public class AutonHasStartingPosition {
  @Test
  public void exampleTest() {
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    System.out.println("Auto List: " + autoNames);

    for (String autoName : autoNames) {
      try {
        Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);
        Assertions.assertNotNull(pose);
        System.out.println("Auto: " + autoName + " Starting Position: " + pose);
      } catch (Exception e) {
        System.out.println("Auto: " + autoName + " No starting pose!!!");
        Assertions.fail();
      }
    }
  }
}
