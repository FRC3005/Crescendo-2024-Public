package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tinylog.Logger;
import org.tinylog.configuration.Configuration;

public class LogConfig {
  // Don't show class or function names as this has an impact on performance
  // see https://tinylog.org/v2/benchmark/
  private static final String kFormat =
      "[UserLog] {date:yyyy-MM-dd HH:mm:ss} - {tag: No Tag} - {level}: {message}";

  /** Options are: trace debug info warn error */

  // Logging to DS console
  private static final String kConsoleLevel = "trace";

  // Logging to USB
  private static final String kLogLevel = "trace";

  public static void config() {
    Configuration.set("writer1", "frc.lib.logging.tinylog.DataLogWriter");
    Configuration.set("writer1.level", kLogLevel);
    Configuration.set("writer1.format", kFormat);

    Configuration.set("writer2", "org.tinylog.writers.ConsoleWriter");
    Configuration.set("writer2.level", kConsoleLevel);
    Configuration.set("writer2.format", kFormat);

    CommandScheduler.getInstance()
        .onCommandFinish(
            (cmd) -> Logger.tag("Scheduler").trace("Finish Command: {}", cmd.getName()));
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (cmd) -> Logger.tag("Scheduler").trace("Initialize Command: {}", cmd.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (cmd) -> Logger.tag("Scheduler").trace("Interrupted Command: {}", cmd.getName()));
  }
}
