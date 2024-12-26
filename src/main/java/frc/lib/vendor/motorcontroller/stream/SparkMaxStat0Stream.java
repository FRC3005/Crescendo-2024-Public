package frc.lib.vendor.motorcontroller.stream;

import frc.lib.telemetrystream.DoubleSample;
import frc.lib.telemetrystream.IntegerSample;
import frc.lib.telemetrystream.StreamedFrame;
import java.util.function.Consumer;

public class SparkMaxStat0Stream implements StreamedFrame {
  private final SparkMaxFrames.Status0In frame = new SparkMaxFrames.Status0In();
  private Consumer<DoubleSample> m_appliedOutputConsumer = null;
  private Consumer<IntegerSample> m_stickyFaultsConsumer = null;

  public SparkMaxStat0Stream appliedOutputConsumer(Consumer<DoubleSample> consumer) {
    m_appliedOutputConsumer = consumer;
    return this;
  }

  public SparkMaxStat0Stream stickyFaultsConsumer(Consumer<IntegerSample> consumer) {
    m_stickyFaultsConsumer = consumer;
    return this;
  }

  @Override
  public void processOnMatch(int arbid, long timestamp, byte[] data, int length) {
    if (((arbid & 0x1FFFFFC0) != 0x2051800) || length < 8) {
      return;
    }

    frame.Deserialize(data);

    if (m_appliedOutputConsumer != null) {
      m_appliedOutputConsumer.accept(
          new DoubleSample(timestamp, arbid & 0x3F, frame.getAppliedOutput()));
    }

    if (m_stickyFaultsConsumer != null) {
      m_stickyFaultsConsumer.accept(
          new IntegerSample(timestamp, arbid & 0x3F, (int) frame.stickyFaults));
    }
  }
}
