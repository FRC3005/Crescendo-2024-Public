package frc.lib.vendor.motorcontroller.stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.lib.vendor.motorcontroller.stream.SparkMaxFrames.Status0In;
import frc.lib.vendor.motorcontroller.stream.SparkMaxFrames.Status1In;
import frc.lib.vendor.motorcontroller.stream.SparkMaxFrames.Status2In;
import org.junit.jupiter.api.*;

public class SparkMaxFramesTest {

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void stat0Frame() {
    // Create and set up the original object
    Status0In original = new Status0In();
    original.appliedOutput = 100;
    original.faults = 200;
    original.stickyFaults = 300;
    original.sensorInv = 1;
    original.setpointInv = 0;
    original.mtrType = 1;
    original.isFollower = 0;

    // Serialize the object
    byte[] serializedData = original.Serialize();

    // Deserialize into a new object
    Status0In deserialized = new Status0In();
    deserialized.Deserialize(serializedData);

    // Assert that the original and deserialized objects are equal
    assertEquals(original.appliedOutput, deserialized.appliedOutput);
    assertEquals(original.faults, deserialized.faults);
    assertEquals(original.stickyFaults, deserialized.stickyFaults);
    assertEquals(original.sensorInv, deserialized.sensorInv);
    assertEquals(original.setpointInv, deserialized.setpointInv);
    assertEquals(original.mtrType, deserialized.mtrType);
    assertEquals(original.isFollower, deserialized.isFollower);
  }

  @Test
  void stat1FrameTest1() {
    // Create and set up the original object
    Status1In original = new Status1In();
    original.sensorVel = 123.456f;
    original.mtrTemp = 78;
    original.mtrVoltage = 3065;
    original.mtrCurrent = 4005;

    // Serialize the object
    byte[] serializedData = original.Serialize();

    // Deserialize into a new object
    Status1In deserialized = new Status1In();
    deserialized.Deserialize(serializedData);

    // Assert that the original and deserialized objects are equal
    assertEquals(original.sensorVel, deserialized.sensorVel);
    assertEquals(original.mtrTemp, deserialized.mtrTemp);
    assertEquals(original.mtrVoltage, deserialized.mtrVoltage);
    assertEquals(original.mtrCurrent, deserialized.mtrCurrent);
  }

  @Test
  void stat1FrameTest2() {
    Status1In original = new Status1In();
    original.setVelocity(123.456);
    original.mtrTemp = 78;
    // Pick voltage and current values closer to max for the frame
    original.setBusVoltage(30.15);
    original.setOutputCurrent(125.12);

    // Serialize the object
    byte[] serializedData = original.Serialize();

    // Deserialize into a new object
    Status1In deserialized = new Status1In();
    deserialized.Deserialize(serializedData);

    // Assert that the original and deserialized objects are equal
    assertEquals(original.getVelocity(), deserialized.getVelocity());
    assertEquals(original.mtrTemp, deserialized.mtrTemp);
    assertEquals(original.getBusVoltage(), deserialized.getBusVoltage());
    assertEquals(original.getOutputCurrent(), deserialized.getOutputCurrent());
  }

  @Test
  void stat1FrameTest3() {
    // Create and set up the original object
    Status1In original = new Status1In();
    original.sensorVel = 123.456f;
    original.mtrTemp = 78;
    original.mtrVoltage = 1234;
    original.mtrCurrent = 50;

    // Serialize the object
    byte[] serializedData = original.Serialize();

    // Deserialize into a new object
    Status1In deserialized = new Status1In();
    deserialized.Deserialize(serializedData);

    // Assert that the original and deserialized objects are equal
    assertEquals(original.sensorVel, deserialized.sensorVel);
    assertEquals(original.mtrTemp, deserialized.mtrTemp);
    assertEquals(original.mtrVoltage, deserialized.mtrVoltage);
    assertEquals(original.mtrCurrent, deserialized.mtrCurrent);
  }

  @Test
  void stat2Frame() {
    // Create and set up the original object
    Status2In original = new Status2In();
    original.sensorPos = 123.456f; // example value
    original.iAccum = 789.012f; // example value

    // Serialize the object
    byte[] serializedData = original.Serialize();

    // Deserialize into a new object
    Status2In deserialized = new Status2In();
    deserialized.Deserialize(serializedData);

    // Assert that the original and deserialized objects are equal
    assertEquals(
        original.sensorPos, deserialized.sensorPos, 0.001); // Float comparison with a delta
    assertEquals(original.iAccum, deserialized.iAccum, 0.001); // Float comparison with a delta
  }
}
