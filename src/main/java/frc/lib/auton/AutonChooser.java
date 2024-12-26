package frc.lib.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.tinylog.Logger;

public class AutonChooser {
  private static final HashMap<String, Command> m_autons = new HashMap<>();
  private static final List<String> m_autoList = new ArrayList<>();
  private static boolean m_isInit = false;

  private static Command m_defaultAuton = new WaitCommand(0.0).withName("Do Nothing Default");

  static {
    addAuton(new WaitCommand(0.0).withName("Do Nothing Auton"), "Do Nothing");
  }

  /**
   * Add an auton to the available autons.
   *
   * @param auton Auton command to run.
   * @param name Name of the command as displayed in the auton list.
   */
  public static void addAuton(Command auton, String name) {
    if (m_autons.containsKey(name)) {
      Logger.tag("Auton Chooser").error("Auton already exists with name {}!", name);
      return;
    }

    m_autons.put(name, auton);
    m_autoList.add(name);

    String[] autonList = new String[m_autoList.size()];
    autonList = m_autoList.toArray(autonList);
    SmartDashboard.putStringArray("Auto List", autonList);

    if (!m_isInit) {
      m_isInit = true;
      String existingCommand = SmartDashboard.getString("Auto Selector", "");
      SmartDashboard.putString("Auto Selector", existingCommand);
    }
  }

  /**
   * Get the selected auton, or the default if not found.
   *
   * @return Auton command to run.
   */
  public static Command getAuton() {
    String selectedAuton =
        SmartDashboard.getString("Auto Selector", "INVALID VALUE FROM NETWORK TABLES");
    if (!m_autons.containsKey(selectedAuton)) {
      Logger.tag("Auton Chooser")
          .error(
              "Auton selection {} not found, falling back to {}",
              selectedAuton,
              m_defaultAuton.getName());
    }
    var auto = m_autons.getOrDefault(selectedAuton, m_defaultAuton);
    Logger.tag("AutonChooser").info("Get Auton {} -- Auton name {}", selectedAuton, auto.getName());

    return auto;
  }

  /**
   * Get a list of registered auton names.
   *
   * @return list of auton names.
   */
  public static List<String> getRegisteredAutons() {
    return m_autoList;
  }

  /**
   * Set a default auton to run if none is selected, or if there is an error selecting an auton.
   *
   * @param defaultAuton auton command to set as default.
   */
  public static void setDefaultAuton(Command defaultAuton) {
    m_defaultAuton = defaultAuton;
  }
}
