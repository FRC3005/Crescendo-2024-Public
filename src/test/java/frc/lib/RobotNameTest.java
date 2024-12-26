package frc.lib;

import frc.lib.util.RobotName;
import org.junit.jupiter.api.*;

public class RobotNameTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void exampleTest() {
    int uninitialized = RobotName.select(100, 10);
    Assertions.assertEquals(100, uninitialized);

    RobotName.set(RobotName.Name.Practice);
    int myValue = RobotName.select(100, 10);
    Assertions.assertEquals(10, myValue);

    RobotName.set(RobotName.Name.Competition);
    int myValue2 = RobotName.select(100, 10);
    Assertions.assertEquals(100, myValue2);
  }
}
