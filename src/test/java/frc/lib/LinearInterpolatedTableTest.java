package frc.lib;

import frc.lib.util.LinearInterpolatedTable2d;
import org.junit.jupiter.api.*;

public class LinearInterpolatedTableTest {
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
    // spotless:off
    LinearInterpolatedTable2d table = new LinearInterpolatedTable2d()
      .withPair(0.0, 1.0)
      .withPair(1.0, 3.0)
      .withPair(0.5, 2.0)
      .withPair(2.0, 5.0);
    // spotless:on

    System.out.println(table);
    Assertions.assertEquals(3.2, table.get(1.1), kEpsilon);
    Assertions.assertEquals(3.0, table.get(1.0), kEpsilon);
    Assertions.assertEquals(4.0, table.get(1.5), kEpsilon);
    Assertions.assertEquals(7.0, table.get(3.0), kEpsilon);
    Assertions.assertEquals(1.0, table.get(0.0), kEpsilon);
    Assertions.assertEquals(-1.0, table.get(-1.0), kEpsilon);
    Assertions.assertEquals(-3.0, table.get(-2.0), kEpsilon);
  }
}
