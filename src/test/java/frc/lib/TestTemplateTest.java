package frc.lib;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;

public class TestTemplateTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  @Test
  public void exampleTest() {
    int myTestValue = 4;

    double myDoubleValue = 17.17 / 3005.0;
    // Run all assertions even if they fail.
    Assertions.assertTrue(true, "This passes if true");
    Assertions.assertEquals(
        4, myTestValue, "This passes if these two are equal, expected goes 2nd");
    Assertions.assertEquals(
        0.00571381, myDoubleValue, kEpsilon, "Doubles don't have infinite precision");
  }
}
