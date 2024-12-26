package frc.lib.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import java.util.ArrayList;
import java.util.List;

public class TelemetryRunner {
  private static TelemetryRunner s_default = null;
  private NetworkTable m_table;
  private List<TelemetryBuilder> m_builders = new ArrayList<>();

  private TelemetryRunner() {
    this("Telemetry");
  }

  public TelemetryRunner(String rootKey) {
    m_table = NetworkTableInstance.getDefault().getTable(rootKey);
  }

  public static TelemetryRunner getDefault() {
    if (s_default == null) {
      s_default = new TelemetryRunner();
    }
    return s_default;
  }

  public void bind(TelemetryNode node) {
    var builder = new TelemetryBuilderImpl();
    builder.setTable(m_table.getSubTable(node.getClass().getSimpleName()));
    node.bind(builder);
    builder.update();
    m_builders.add(builder);
  }

  public void bindSendable(Sendable sendable) {
    var builder = new TelemetryBuilderImpl();
    builder.setTable(m_table.getSubTable(sendable.getClass().getSimpleName()));
    sendable.initSendable(builder);
    builder.update();
    m_builders.add(builder);
  }

  public void update() {
    for (var builder : m_builders) {
      builder.update();
    }
  }
}
