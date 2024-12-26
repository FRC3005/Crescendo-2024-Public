package frc.lib;

import edu.wpi.first.hal.HAL;
import frc.lib.util.AccessOrderHashSet;
import org.junit.jupiter.api.*;

public class AccessOrderHashSetTest {
  @BeforeEach // this method will run before each test
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void insertTest() {
    AccessOrderHashSet<Integer> hs = new AccessOrderHashSet<>();
    hs.add(1);
    hs.add(10);
    hs.add(5);
    var it = hs.iterator();

    Assertions.assertEquals(1, it.next());
    Assertions.assertEquals(10, it.next());
    Assertions.assertEquals(5, it.next());
    return;
  }

  @Test
  public void insertAccessTest() {
    AccessOrderHashSet<Integer> hs = new AccessOrderHashSet<>();
    hs.add(1);
    hs.add(10);
    hs.add(5);
    hs.add(10);

    var it = hs.iterator();
    Assertions.assertEquals(1, it.next());
    Assertions.assertEquals(5, it.next());
    Assertions.assertEquals(10, it.next());
    Assertions.assertEquals(3, hs.size());
    return;
  }
}
