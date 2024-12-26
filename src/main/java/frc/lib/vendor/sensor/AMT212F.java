package frc.lib.vendor.sensor;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.electromechanical.AbsoluteEncoder;
import frc.lib.telemetry.TelemetryBuilder;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class AMT212F implements AbsoluteEncoder {
  /**
   * Create a new AMT212 sensor. This class creates a thread to read the sensor over UART in the
   * background. The default rate is 200Hz, but is nowhere near exact as it uses thread.sleep().
   *
   * @param baud sensor baud rate configured in the device
   * @param offset added to the sensor value after the scalar is applied
   * @param scalar multiplied by the range [0, 1] read by the sensor
   * @param invert change sensor polarity
   */
  public AMT212F(Baud baud, double offset, double scalar, boolean invert) {
    m_serial = new SerialPort(baud.value, Port.kMXP, 8, Parity.kNone, StopBits.kOne);
    m_serial.disableTermination();
    m_serial.setTimeout(0.001);
    m_serial.setReadBufferSize(20);
    m_serial.setWriteBufferMode(WriteBufferMode.kFlushWhenFull);
    m_serial.setWriteBufferSize(1);
    m_timer.restart();

    m_offsetInSensorUnits = (int) ((offset / scalar) * kMaxIntSensorValue);
    m_scalar = scalar;
    m_invert = invert;

    m_thread = new Thread(this::threadMain, "AMT212F");
    m_thread.start();
  }

  public AMT212F(Baud baud) {
    this(baud, 0.0, 1.0, false);
  }

  private void threadMain() {
    boolean allowUpdates = false;
    while (m_keepRunning.get()) {
      if (m_timer.hasElapsed(kConnectionTimeoutSeconds)) {
        allowUpdates = false;
        m_isConnected.set(false);
      } else {
        allowUpdates = true;
        m_isConnected.set(true);
      }

      m_serial.reset();
      Timer.delay(0.001);
      m_serial.write(kReadPositionByte, 1);
      m_serial.flush();

      // Set 0 timeout and use this to throttle the thread
      Timer.delay(0.004);

      // Always clear buffer
      int bytesToRead = m_serial.getBytesReceived();
      byte[] values = m_serial.read(bytesToRead);

      if (bytesToRead < 2 || values == null || values.length < 2) {
        m_errorCount.incrementAndGet();
        m_underflowCount.incrementAndGet();
        continue;
      }

      if (bytesToRead > 2 || values.length > 2) {
        m_errorCount.incrementAndGet();
        m_overflowCount.incrementAndGet();
        continue;
      }

      ByteBuffer buffer = ByteBuffer.wrap(values);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      int result;
      try {
        result = buffer.getShort();
      } catch (Exception e) {
        m_errorCount.incrementAndGet();
        continue;
      }

      if (!calculateOddEvenParity(result)) {
        m_parityErrorCount.incrementAndGet();
        m_errorCount.incrementAndGet();
      } else {
        // Valid data recieved
        result &= 0x3FFF;
        if (allowUpdates) {
          m_atomicValue.set(result);
        }
        m_timer.restart();
      }

      if (m_setZero.getAndSet(false)) {
        m_serial.write(kResetEncoderBytes, kResetEncoderBytes.length);
      }
    }
  }

  private boolean calculateOddEvenParity(int value) {
    boolean evenParity = false;
    boolean oddParity = false;
    for (int i = 0; i < 8; i++) {
      boolean even = (value & 1) != 0;
      boolean odd = (value & 2) != 0;

      value = value >> 2;

      evenParity ^= even;
      oddParity ^= odd;
    }

    return oddParity && evenParity;
  }

  /** Set the raw sensor zero. This will be the offset value when using a non-0 offset value. */
  public void setZero() {
    m_setZero.set(true);
  }

  public double get() {
    int sensorValue = m_atomicValue.get();

    if (m_invert) {
      sensorValue = kMaxIntSensorValue - sensorValue;
    }

    sensorValue += m_offsetInSensorUnits;

    if (sensorValue < 0) {
      sensorValue = kMaxIntSensorValue + sensorValue;
    } else if (sensorValue >= kMaxIntSensorValue) {
      sensorValue = sensorValue - kMaxIntSensorValue;
    }

    double convertedValue = ((sensorValue / kMaxSensorValue) * m_scalar);

    return convertedValue;
  }

  @Override
  public boolean isConnected() {
    return m_isConnected.get();
  }

  public enum Baud {
    k9600(9600),
    k19200(19200),
    k38400(38400),
    k115200(115200);

    public final int value;

    private Baud(int baud) {
      this.value = baud;
    }
  }

  private final SerialPort m_serial;
  private Thread m_thread;
  private Timer m_timer = new Timer();
  private AtomicInteger m_atomicValue = new AtomicInteger(0);
  private AtomicBoolean m_setZero = new AtomicBoolean(false);
  private AtomicBoolean m_keepRunning = new AtomicBoolean(true);
  private AtomicBoolean m_isConnected = new AtomicBoolean(false);
  private AtomicInteger m_errorCount = new AtomicInteger(0);
  private AtomicInteger m_underflowCount = new AtomicInteger(0);
  private AtomicInteger m_overflowCount = new AtomicInteger(0);
  private AtomicInteger m_parityErrorCount = new AtomicInteger(0);

  private final double kConnectionTimeoutSeconds = 0.05;

  private final byte[] kReadPositionByte = new byte[] {0x54};
  private final byte[] kResetEncoderBytes = new byte[] {0x56, 0x5E};

  private final int m_offsetInSensorUnits;
  private final double m_scalar;
  private final boolean m_invert;

  private final double kMaxSensorValue = 16384.0;
  private final int kMaxIntSensorValue = 16383;

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.addBooleanProperty("IsConnected", this::isConnected, null);
    builder.addIntegerProperty("ErrorCount", () -> m_errorCount.get(), null);
    builder.addIntegerProperty("Error/BufferUnderflow", () -> m_underflowCount.get(), null);
    builder.addIntegerProperty("Error/BufferOverflow", () -> m_overflowCount.get(), null);
    builder.addIntegerProperty("Error/ParityError", () -> m_parityErrorCount.get(), null);
    builder.addDoubleProperty("Value", this::get, null);
    builder.addBooleanProperty(
        "SetZero",
        () -> false,
        (val) -> {
          if (val) setZero();
        });
  }

  @Override
  public double getPosition() {
    return get();
  }
}
