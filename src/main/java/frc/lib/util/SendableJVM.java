package frc.lib.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableJVM implements Sendable {
  private final String runtimeVersion = Runtime.version().toString();

  @Override
  public void initSendable(SendableBuilder builder) {
    Runtime runtime = Runtime.getRuntime();
    if (runtime != null) {
      builder.addDoubleProperty("Processors", () -> (double) runtime.availableProcessors(), null);
      builder.addDoubleProperty("Free Memory", () -> (double) runtime.freeMemory(), null);
      builder.addBooleanProperty(
          "Run GC",
          () -> false,
          (val) -> {
            if (val) {
              runtime.gc();
            }
          });
      builder.addDoubleProperty("Max Memory", () -> (double) runtime.maxMemory(), null);
      builder.addDoubleProperty("Total Memory", () -> (double) runtime.totalMemory(), null);
      builder.addStringProperty("Version", () -> runtimeVersion, null);
    }
  }
}
