package frc.lib.vendor.motorcontroller.stream;

import edu.wpi.first.hal.CANStreamMessage;
import frc.lib.telemetrystream.DoubleSample;
import frc.lib.telemetrystream.IntegerSample;
import frc.lib.telemetrystream.TelemetryStream;
import frc.lib.vendor.motorcontroller.stream.SparkMaxFrames.DataFrame;
import java.util.function.Consumer;

public class SparkMaxStream {
  private final TelemetryStream m_stream;

  // See REV API v1.x for details, or reverse engineer messages using hardware client
  // Grab all REV motor controller frames with API Class 6
  // 0x2051800 | ([Periodic frame number] << 6)
  private static final int REV_API_ID = 0x2051800;

  // Match all fields except API id
  private static final int REV_API_MASK = 0x1FFFFC3F;

  // Set default buffer size. Assume max telemetry frames = 8,
  // fastest rate = 5ms, polling rateworst case (with margin) = 40ms
  // (40 / 5) * 8 = 64
  private static final int STEAM_BUFFER_SIZE = 64;

  private final SparkMaxStat0Stream m_stat0Stream = new SparkMaxStat0Stream();
  private final SparkMaxStat1Stream m_stat1Stream = new SparkMaxStat1Stream();
  private final SparkMaxStat2Stream m_stat2Stream = new SparkMaxStat2Stream();

  private final int m_deviceId;

  /**
   * Create a Spark Max stream object. The stream will accept frames from stat0, stat1, stat2 and
   * expose callbacks to operate on the data.
   */
  public SparkMaxStream(int deviceId) {
    // All defaults, including reading from all devices. (Pass device arsing
    // logic down to callback...)
    m_deviceId = deviceId;
    m_stream = new TelemetryStream(REV_API_ID | deviceId, REV_API_MASK, STEAM_BUFFER_SIZE);
    m_stream.addFrame(m_stat0Stream);
    m_stream.addFrame(m_stat1Stream);
    m_stream.addFrame(m_stat2Stream);
  }

  /** Start the stream */
  public void start() {
    m_stream.start();
  }

  /** Stop the stream */
  public void stop() {
    m_stream.stop();
  }

  public void poll() {
    m_stream.poll();
  }

  // Lazy way to test...
  protected void injectAndPoll(DataFrame[] frames, long timestamp) {
    CANStreamMessage[] messages = new CANStreamMessage[frames.length];

    for (int frameNum = 0; frameNum < frames.length; frameNum++) {
      byte[] frameData = frames[frameNum].Serialize();
      messages[frameNum] = new CANStreamMessage();
      for (int i = 0; i < frameData.length; i++) {
        messages[frameNum].data[i] = frameData[i];
      }
      messages[frameNum].messageID = frames[frameNum].arbId(m_deviceId);
      messages[frameNum].length = frameData.length;
      messages[frameNum].timestamp = timestamp;
    }
    m_stream.injectAndPoll(messages);
  }

  /**
   * Callback for applied output frames.
   *
   * @param consumer a Streamed Double consumer, which includes timestamp, device id, and data for
   *     all recieved applied outputs.
   * @return this
   */
  public SparkMaxStream appliedOutput(Consumer<DoubleSample> consumer) {
    m_stat0Stream.appliedOutputConsumer(consumer);
    return this;
  }

  /**
   * Callback for sticky fauls.
   *
   * @param consumer a Streamed Integer consumer, which includes timestamp, device id, and data for
   *     all recieved sticky faults.
   * @return this
   */
  public SparkMaxStream stickyFaults(Consumer<IntegerSample> consumer) {
    m_stat0Stream.stickyFaultsConsumer(consumer);
    return this;
  }

  /**
   * Callback for velocity.
   *
   * @param consumer a Streamed Double consumer, which includes timestamp, device id, and data for
   *     all recieved velocity updates.
   * @return this
   */
  public SparkMaxStream velocity(Consumer<DoubleSample> consumer) {
    m_stat1Stream.velocityConsumer(consumer);
    return this;
  }

  /**
   * Callback for output current.
   *
   * @param consumer a Streamed Double consumer, which includes timestamp, device id, and data for
   *     all recieved output current updates.
   * @return this
   */
  public SparkMaxStream current(Consumer<DoubleSample> consumer) {
    m_stat1Stream.currentConsumer(consumer);
    return this;
  }

  /**
   * Callback for supply voltage.
   *
   * @param consumer a Streamed Double consumer, which includes timestamp, device id, and data for
   *     all recieved supply voltage updates.
   * @return this
   */
  public SparkMaxStream supplyVoltage(Consumer<DoubleSample> consumer) {
    m_stat1Stream.supplyVoltageConsumer(consumer);
    return this;
  }

  /**
   * Callback for position.
   *
   * @param consumer a Streamed Double consumer, which includes timestamp, device id, and data for
   *     all recieved position updates.
   * @return this
   */
  public SparkMaxStream position(Consumer<DoubleSample> consumer) {
    m_stat2Stream.positionConsumer(consumer);
    return this;
  }
}
