package frc.lib;

import frc.lib.electromechanical.AbsoluteEncoder;
import frc.lib.electromechanical.OffsetAbsoluteEncoder;
import org.junit.jupiter.api.*;

public class OffsetAbsoluteEncoderTest {
  private static final double kEpsilon = 1E-9;

  public static class DummyAbsolute implements AbsoluteEncoder {
    public double position;

    @Override
    public double getPosition() {
      return position;
    }
  }

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void exampleTest() {
    DummyAbsolute absoluteSensor = new DummyAbsolute();
    OffsetAbsoluteEncoder offsetSensor =
        new OffsetAbsoluteEncoder(absoluteSensor, 90.0, 0.0, 360.0);

    absoluteSensor.position = 0.0;
    Assertions.assertEquals(270.0, offsetSensor.getPosition(), kEpsilon);
    absoluteSensor.position = 90.0;
    Assertions.assertEquals(0.0, offsetSensor.getPosition(), kEpsilon);
    absoluteSensor.position = 90.1;
    Assertions.assertEquals(0.1, offsetSensor.getPosition(), kEpsilon);
    absoluteSensor.position = 89.9;
    Assertions.assertEquals(359.9, offsetSensor.getPosition(), kEpsilon);
    absoluteSensor.position = 180.0;
    Assertions.assertEquals(90.0, offsetSensor.getPosition(), kEpsilon);
  }
}
