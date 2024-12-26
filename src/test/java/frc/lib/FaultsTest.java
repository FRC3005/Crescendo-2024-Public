package frc.lib;

import frc.lib.util.Faults;
import org.junit.jupiter.api.*;

public class FaultsTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  private static String expectedString = "";
  private static Boolean fatalFault = false;
  private static Boolean faultPresent = false;
  private static int calls = 0;

  private static void runTest(Faults.Fault fault) {
    Assertions.assertEquals(expectedString, fault.message());
    Assertions.assertEquals(fatalFault, fault.fatal());
    Assertions.assertEquals(faultPresent, fault.present());
    calls++;
  }

  @Test
  public void exampleTest() {
    expectedString = "Some message";
    fatalFault = false;
    faultPresent = true;
    Faults.onFault(FaultsTest::runTest);
    Faults.subsystem("Arm").error(expectedString);

    Assertions.assertEquals(1, Faults.count());
    Assertions.assertEquals(1, calls);

    expectedString = "";
    faultPresent = false;
    Faults.subsystem("Arm").clear();
    Assertions.assertEquals(0, Faults.count());

    expectedString = "Some message";
    faultPresent = true;
    Faults.subsystem("Arm").error(expectedString);
    Faults.subsystem("Arm").error(expectedString);

    Assertions.assertEquals(1, Faults.count());
    Assertions.assertEquals(4, calls);

    expectedString = "";
    faultPresent = false;
    Faults.clearAll();
    Assertions.assertEquals(0, Faults.count());

    expectedString = "Something else";
    fatalFault = false;
    faultPresent = true;
    Faults.subsystem("Arm").error(expectedString);
    expectedString = "also something else";
    fatalFault = true;
    faultPresent = true;
    Faults.subsystem("Leg").fatal(expectedString);
    Assertions.assertEquals(7, calls);
    Assertions.assertEquals(2, Faults.count());
    Faults.subsystem("Leg").clear();

    expectedString = "";
    fatalFault = false;
    faultPresent = false;
    Faults.subsystem("Arm").clear();
    Assertions.assertEquals(1, Faults.count());

    expectedString = "Something else";
    fatalFault = false;
    faultPresent = true;
    Faults.subsystem("Arm").error(expectedString);
    calls = 0;

    expectedString = "";
    fatalFault = false;
    faultPresent = false;
    Faults.subsystem("Arm").clear();
    Faults.subsystem("Arm").clear();
    Faults.subsystem("Arm").clear();
    Faults.subsystem("Arm").clear();
    Assertions.assertEquals(1, calls);
  }
}
