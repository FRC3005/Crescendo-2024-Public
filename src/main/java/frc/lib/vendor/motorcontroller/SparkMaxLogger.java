package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import frc.lib.telemetrystream.DoubleSample;
import frc.lib.telemetrystream.IntegerSample;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStream;
import java.util.function.Consumer;

public class SparkMaxLogger {
  private final SparkMaxStream m_stream;
  private static final String NT_PREFIX = "/hslog/sparkmax/";
  private final int m_deviceId;
  private final DataLog m_datalog;
  private final String m_name;
  private static long m_timescaleDelta = 0;

  public static void calculateTimescaleDelta() {
    m_timescaleDelta = (System.nanoTime() / 1000) - HALUtil.getFPGATime();
  }

  private Consumer<DoubleSample> LoggedStreamDouble(String ntEntryName) {
    String fullNtPath = NT_PREFIX + m_name + "/" + ntEntryName;
    DoubleLogEntry logEntry = new DoubleLogEntry(m_datalog, fullNtPath);
    return (streamedDouble) -> {
      logEntry.append(streamedDouble.value, streamedDouble.timestamp * 1000 - m_timescaleDelta);
    };
  }

  private Consumer<IntegerSample> LoggedIntegerStream(String ntEntryName) {
    String fullNtPath = NT_PREFIX + m_name + "/" + ntEntryName;
    IntegerLogEntry logEntry = new IntegerLogEntry(m_datalog, fullNtPath);
    return (streamedDouble) -> {
      logEntry.append(streamedDouble.value, streamedDouble.timestamp * 1000 - m_timescaleDelta);
    };
  }

  public SparkMaxLogger(CANSparkMax sparkMax, DataLog log) {
    this(sparkMax, String.valueOf(sparkMax.getDeviceId()), log);
  }

  public SparkMaxLogger(CANSparkMax sparkMax, String deviceName, DataLog log) {
    m_deviceId = sparkMax.getDeviceId();
    m_datalog = log;
    m_name = deviceName;
    calculateTimescaleDelta();
    m_stream =
        new SparkMaxStream(m_deviceId)
            .appliedOutput(LoggedStreamDouble("appliedOutput"))
            .stickyFaults(LoggedIntegerStream("stickyFaults"))
            .current(LoggedStreamDouble("outputCurrent"))
            .velocity(LoggedStreamDouble("velocity"))
            .position(LoggedStreamDouble("position"))
            .supplyVoltage(LoggedStreamDouble("supplyVoltage"));
  }

  public void start() {
    m_stream.start();
  }

  public void stop() {
    m_stream.stop();
  }

  public void poll() {
    m_stream.poll();
  }
}
