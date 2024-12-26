package frc.lib.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import org.tinylog.Logger;

public abstract class AutonCommandBase extends SequentialCommandGroup implements TelemetryNode {
  private double m_autonStartTime = 0.0;
  private String m_currentCommand = "";
  private double m_autonEndTime = 0.0;
  private final String m_name;

  public AutonCommandBase() {
    this(new String());
  }

  public AutonCommandBase(String name) {
    if (name.isEmpty()) {
      name = this.getClass().getSimpleName();
      m_name = name.substring(name.lastIndexOf('.') + 1);
    } else {
      m_name = name;
    }

    Logger.tag("Auton").info("Adding Auton {}", name);
    AutonChooser.addAuton(this, name);
  }

  public void addCommandsWithLog(Command... commands) {
    this.addCommandsWithLog("Auton " + m_name, commands);
  }

  public void addCommandsWithLog(String tag, Command... commands) {
    // Add logging to each command

    // Log start of group
    super.addCommands(
        new InstantCommand(
            () -> {
              m_autonStartTime = Timer.getFPGATimestamp();
              Logger.tag(tag).trace("Auton goup Starting at {}", m_autonStartTime);
            }));

    // Log start and end of each command
    for (var cmd : commands) {
      super.addCommands(
          new InstantCommand(
              () -> {
                m_currentCommand = cmd.getName();
                Logger.tag(tag)
                    .trace(
                        "Starting command step {}, at {}",
                        m_currentCommand,
                        Timer.getFPGATimestamp());
              }),
          cmd,
          new InstantCommand(
              () ->
                  Logger.tag(tag)
                      .trace(
                          "Ending command step {}, at {}",
                          cmd.getName(),
                          Timer.getFPGATimestamp())));
    }

    // Log end of auton
    super.addCommands(
        new InstantCommand(
            () -> {
              m_autonEndTime = Timer.getFPGATimestamp();
              Logger.tag(tag)
                  .trace(
                      "Auton group complete after {} seconds", m_autonEndTime - m_autonStartTime);
              m_currentCommand = "";
            }));
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Current Command", () -> m_currentCommand, null);
    builder.addDoubleProperty("Auton Start Time", () -> m_autonStartTime, null);
    builder.addStringProperty("Auton Name", () -> m_name, null);
  }
}
