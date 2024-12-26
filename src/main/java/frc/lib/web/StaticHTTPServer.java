package frc.lib.web;

import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Path;
import java.nio.file.Paths;

public class StaticHTTPServer {

  public static void startServer(Path filePath) {
    // Start server
    // Kudos Mechanical-Advantage
    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/objectivetracker/NodeSelectorIOServer.java
    var app =
        Javalin.create(
            config -> {
              config.staticFiles.add(
                  Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "www")
                      .toString(),
                  Location.EXTERNAL);
            });
    app.start(5800);
  }
}
