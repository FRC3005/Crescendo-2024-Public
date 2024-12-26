package frc.lib.telemetrystream;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.stream.SparkMaxFrames;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStat0Stream;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStat1Stream;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStat2Stream;
import java.util.ArrayList;
import org.junit.jupiter.api.*;

public class TelemetryStreamTest {
  private static final double kEpsilon = 1E-9;

  // See REV API v1.x for details, or reverse engineer messages using hardware client
  // Grab all REV motor controller frames with API Class 6
  // 0x2051800 | ([Periodic frame number] << 6)
  private static final int REV_API_ID = 0x2050000 | (0x6 << 10);

  // Match all fields except API Index
  private static final int REV_API_MASK = 0x1FFFFC3F;

  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  private CANStreamMessage setMessage(int messageId, byte[] data, long timestamp) {
    var result = new CANStreamMessage();
    for (int i = 0; i < data.length; i++) {
      result.data[i] = data[i];
    }
    result.length = data.length;
    result.messageID = messageId;
    result.timestamp = timestamp;

    return result;
  }

  @Test
  public void exampleTest() {
    ArrayList<Double> velocityValues = new ArrayList<>();
    ArrayList<Double> currentValues = new ArrayList<>();
    ArrayList<Double> appliedValues = new ArrayList<>();
    ArrayList<Double> positionValues = new ArrayList<>();
    ArrayList<Long> velocityTimestamps = new ArrayList<>();
    ArrayList<Long> currentTimestamps = new ArrayList<>();
    ArrayList<Long> appliedTimestamps = new ArrayList<>();
    ArrayList<Long> positionTimestamps = new ArrayList<>();
    final double[] expectedVelocityValues = {0.1, 1.5, 3005.3005};
    final double[] expectedCurrentValues = {14.8, 11.8, 120.10};
    final double[] expectedAppliedValues = {0.8, -0.56, 0.10};
    final double[] expectedPositionValues = {100.1, 109.1, -40054.2};
    final long[] exectedTimestamps = {1000, 1010, 1022};
    final int canId = 1;
    int fullApiId = REV_API_ID | canId;

    // Setup stream with consumer which test against expected values
    TelemetryStream stream =
        new TelemetryStream(fullApiId, REV_API_MASK)
            .addFrame(
                new SparkMaxStat1Stream()
                    .velocityConsumer(
                        (data) -> {
                          velocityTimestamps.add(data.timestamp);
                          velocityValues.add(data.value);
                          Assertions.assertEquals(1, data.deviceId);
                        })
                    .currentConsumer(
                        (data) -> {
                          currentTimestamps.add(data.timestamp);
                          currentValues.add(data.value);
                          Assertions.assertEquals(1, data.deviceId);
                        }))
            .addFrame(
                new SparkMaxStat0Stream()
                    .appliedOutputConsumer(
                        (data) -> {
                          appliedTimestamps.add(data.timestamp);
                          appliedValues.add(data.value);
                          Assertions.assertEquals(1, data.deviceId);
                        }))
            .addFrame(
                new SparkMaxStat2Stream()
                    .positionConsumer(
                        (data) -> {
                          positionTimestamps.add(data.timestamp);
                          positionValues.add(data.value);
                          Assertions.assertEquals(1, data.deviceId);
                        }));

    stream.start();

    SparkMaxFrames.Status0In stat0 = new SparkMaxFrames.Status0In();
    SparkMaxFrames.Status1In stat1 = new SparkMaxFrames.Status1In();
    SparkMaxFrames.Status2In stat2 = new SparkMaxFrames.Status2In();
    for (int i = 0; i < 3; i++) {
      stat0.setAppliedOutput(expectedAppliedValues[i]);
      stat1.setVelocity(expectedVelocityValues[i]);
      stat1.setOutputCurrent(expectedCurrentValues[i]);
      stat2.setPosition(expectedPositionValues[i]);

      var messages = new ArrayList<CANStreamMessage>();
      messages.add(setMessage(0x0123, new byte[1], 1));
      messages.add(setMessage(stat0.arbId() | canId, stat0.Serialize(), exectedTimestamps[i]));
      messages.add(setMessage(stat1.arbId() | canId, stat1.Serialize(), exectedTimestamps[i]));
      messages.add(setMessage(stat2.arbId() | canId, stat2.Serialize(), exectedTimestamps[i]));

      stream.injectAndPoll(messages.toArray(new CANStreamMessage[4]));
    }

    Assertions.assertArrayEquals(
        expectedAppliedValues,
        appliedValues.stream().mapToDouble(Double::doubleValue).toArray(),
        0.01);

    Assertions.assertArrayEquals(
        expectedCurrentValues,
        currentValues.stream().mapToDouble(Double::doubleValue).toArray(),
        0.1); // less precision due to (Fixed --> floating --> fixed)

    Assertions.assertArrayEquals(
        expectedPositionValues,
        positionValues.stream().mapToDouble(Double::doubleValue).toArray(),
        0.01);

    Assertions.assertArrayEquals(
        expectedVelocityValues,
        velocityValues.stream().mapToDouble(Double::doubleValue).toArray(),
        0.01);
  }
}
