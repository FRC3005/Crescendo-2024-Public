package frc.lib.testmode;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import java.util.ArrayList;
import org.tinylog.Logger;

/*
 * Provide a test mode operation with a few more features:
 *
 * 1) Have an enable/disable for each subsystem. When enabled, run
 *    its periodic function. When its disabled, call its toSafeState()
 *    function.
 *
 * 2) Provide a hook to run a self test for each subsystem. The self test
 *    function will run each subsystem's self test one after another and
 *    provide a report of the results.
 *
 *    TODO: Allow running subsystems in parallel. This can be done by having
 *    a similar dependency system like Command based.
 *
 *    TODO: (Maybe?) allow multiple subsystems to work together for a self test?
 *    In this case, the subsystem itself shouldn't be coupled to the self test
 *    class. Or it could, but also allow passing in just a SelfTest.
 *
 * TODO: Provide a hook to put NT in the same namespace (/TestMode) for each system
 *       instead of in their own.
 *
 */
public class TestMode {
  private static class SubsystemEntry {
    public final TestableSubsystem subsystem;
    public final BooleanEntry enabledEntry;

    public SubsystemEntry(TestableSubsystem subsystem, NetworkTable table) {
      this.subsystem = subsystem;

      String fieldName = subsystem.getName() + " - TestEnable";
      enabledEntry = table.getBooleanTopic(fieldName).getEntry(false);
      enabledEntry.set(false);
    }
  }

  private static void safeAll() {
    s_entries.forEach((entry) -> entry.subsystem.toSafeState());
  }

  // Call in Robot.java testInit()
  public static void init() {
    safeAll();
  }

  // Call in Robot.java disabledInit()
  public static void disabledInit() {
    safeAll();
  }

  // Call in Robot.java testPeriodic()
  public static void periodic() {
    /*
     * Periodic for Test Mode is simply to check if the test mode is enabled,
     * and run its periodic function.
     */
    for (SubsystemEntry entry : s_entries) {
      if (entry.enabledEntry.get()) {
        entry.subsystem.periodic();
      } else {
        entry.subsystem.toSafeState();
      }
    }

    /*
     * Periodic for self test requires a bit more.
     */
  }

  // Register a subsystem with test mode
  public static void register(TestableSubsystem subsystem) {
    initializeNT();
    if (!s_entries.stream().anyMatch((x) -> x.subsystem == subsystem)) {
      Logger.tag("TestModeCore").debug("Adding subsystem {} to TestModes", subsystem.getName());
      s_entries.add(new SubsystemEntry(subsystem, s_table));
    } else {
      Logger.tag("TestModeCore")
          .info("Not re-adding subsystem {} to TestModes, already exists", subsystem.getName());
    }
  }

  private static boolean isInit = false;

  private static void initializeNT() {
    if (isInit) {
      return;
    }
    isInit = true;
    /*
     * Network Tables layout will look something like this:
     * /TestMode/Mode --> One of "Disabled", "Test Mode", "Self Test"
     * /TestMode/ModeStrings -> ["Disabled", "Test Mode", "Self Test"]
     * /TestMode/SelfTest --> Boolean (Run/IsRunning)
     * /TestMode/Subsystems/<Subsystem>_Enable --> Boolean (Enable Test Mode for this subsystem)
     */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    s_table = inst.getTable(kTablePrefix);

    s_table
        .getStringArrayTopic("ModeStrings")
        .publish()
        .set(new String[] {"Disabled", "Test Mode", "Self Test"});
    s_modeSubscriber = s_table.getStringTopic("Mode").getEntry("Disabled");
    s_selfTestState = s_table.getBooleanTopic("SelfTest").getEntry(false);

    s_selfTestState.set(false);
    s_modeSubscriber.set("Disabled");
  }

  private static ArrayList<SubsystemEntry> s_entries = new ArrayList<>();
  private static final String kTablePrefix = "TestMode";
  private static StringEntry s_modeSubscriber;
  private static BooleanEntry s_selfTestState;
  private static NetworkTable s_table;
}
