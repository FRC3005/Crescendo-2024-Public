package frc.lib.util;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.*;

public class JoystickUtilTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void circleDeadbandTest() {
    var result = JoystickUtil.circleDeadband(0.75, 0.0, 0.5);
    Assertions.assertEquals(0.5, result.xAxis, kEpsilon);
    Assertions.assertEquals(0.0, result.yAxis, kEpsilon);

    result = JoystickUtil.circleDeadband(0.0, 0.75, 0.5);
    Assertions.assertEquals(0.0, result.xAxis, kEpsilon);
    Assertions.assertEquals(0.5, result.yAxis, kEpsilon);

    result = JoystickUtil.circleDeadband(-0.75, 0.0, 0.5);
    Assertions.assertEquals(-0.5, result.xAxis, kEpsilon);
    Assertions.assertEquals(0.0, result.yAxis, kEpsilon);

    result = JoystickUtil.circleDeadband(0.0, -0.75, 0.5);
    Assertions.assertEquals(0.0, result.xAxis, kEpsilon);
    Assertions.assertEquals(-0.5, result.yAxis, kEpsilon);

    result =
        JoystickUtil.circleDeadband(
            Math.cos(Units.degreesToRadians(45.0)), Math.sin(Units.degreesToRadians(45.0)), 0.5);
    Assertions.assertEquals(Math.cos(Units.degreesToRadians(45.0)), result.xAxis, kEpsilon);
    Assertions.assertEquals(Math.sin(Units.degreesToRadians(45.0)), result.yAxis, kEpsilon);

    result =
        JoystickUtil.circleDeadband(
            Math.cos(Units.degreesToRadians(-45.0)), Math.sin(Units.degreesToRadians(-45.0)), 0.5);
    Assertions.assertEquals(Math.cos(Units.degreesToRadians(-45.0)), result.xAxis, kEpsilon);
    Assertions.assertEquals(Math.sin(Units.degreesToRadians(-45.0)), result.yAxis, kEpsilon);

    result = JoystickUtil.circleDeadband(0.0, 0.25, 0.5);
    Assertions.assertEquals(0.0, result.xAxis, kEpsilon);
    Assertions.assertEquals(0.0, result.yAxis, kEpsilon);

    result = JoystickUtil.circleDeadband(0.25, 0.25, 0.5);
    Assertions.assertEquals(0.0, result.xAxis, kEpsilon);
    Assertions.assertEquals(0.0, result.yAxis, kEpsilon);
  }
}
