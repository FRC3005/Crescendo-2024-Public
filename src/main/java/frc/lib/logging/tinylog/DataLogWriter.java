package frc.lib.logging.tinylog;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Map;
import org.tinylog.core.LogEntry;
import org.tinylog.writers.AbstractFormatPatternWriter;

public class DataLogWriter extends AbstractFormatPatternWriter {
  StringLogEntry m_stringLog;

  public DataLogWriter(Map<String, String> properties) {
    super(properties);
    m_stringLog = new StringLogEntry(DataLogManager.getLog(), "tinylog");
  }

  @Override
  public void close() throws Exception {
    // No close for this writer
  }

  @Override
  public void flush() throws Exception {
    // No flush for this writer
  }

  @Override
  public void write(LogEntry logEntry) throws Exception {
    m_stringLog.append(render(logEntry));
  }
}
