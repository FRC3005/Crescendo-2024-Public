package frc.lib.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.PrintWriter;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import org.tinylog.Logger;

/**
 * Provide a simple mechanism to define different robots based on a setting stored on the roboRIO
 * itself. This takes the safest approach and defaults to competiton robot.
 *
 * <p>File is stored in the root directory and contains a single name. The name must be defined in
 * this file.
 */
public class RobotName {
  public enum Name {
    Competition(0),
    Practice(1),
    SwerveTestChassis(2);

    public int value;

    private Name(int v) {
      value = v;
    }
  }

  private static Name m_bot = null;

  public static final String kFilename = "/home/lvuser/ROBOT_NAME";

  /**
   * Return true if this is the competition robot. Note that this class will lean towards returning
   * true if there is _any_ kind of error along the selection mechanism. You should log the name of
   * the robot to console.
   *
   * @return true if this is the competition bot
   */
  public static boolean isCompetitionBot() {
    if (m_bot == null) {
      get();
    }

    return m_bot == Name.Competition;
  }

  /**
   * Get the bot declared. This is stored in "/home/lvuser/ROBOT_NAME"
   *
   * @return robot name
   */
  public static Name get() {
    if (m_bot != null) {
      return m_bot;
    }

    if (RobotBase.isReal()) {
      try {
        List<String> lines = Collections.emptyList();
        lines = Files.readAllLines(Paths.get(kFilename), StandardCharsets.UTF_8);
        m_bot = Name.valueOf(lines.get(0).strip());
      } catch (Exception e) {
        provision(Name.Competition);
        m_bot = Name.Competition;
        Logger.tag("RobotName")
            .warn("Bad or no robot name defined, setting to {}", m_bot.toString());
      }
    } else {
      m_bot = Name.Competition;
      Logger.tag("RobotName")
          .warn(
              "No robot name defined, setting to {}. Fix this by calling set() in sim context",
              m_bot.toString());
    }

    return m_bot;
  }

  /**
   * Set the robot name. Use this in simulation or test
   *
   * @param name value set
   */
  public static void set(Name name) {
    if (RobotBase.isReal()) {
      Logger.tag("RobotName")
          .warn("Manually setting robot name... This should be read manually from the device");
    }

    m_bot = name;
  }

  /**
   * Provision a robot with a specific name
   *
   * @param name value to set the robot name to
   */
  public static void provision(Name name) {
    try {
      PrintWriter writer = new PrintWriter(kFilename, "UTF-8");
      writer.println(name.toString());
      writer.close();
    } catch (Exception e) {
      Logger.tag("RobotName").error(e);
    }
  }

  /**
   * Select between competiton or practice bot values. Typically used in Constants.java
   *
   * @param <T> Generic type to select, typically set by the type of the parameters
   * @param compBotValue value to set if this is the competition bot
   * @param practiceBotValue value to set if this is the practice bot
   * @return value to set
   */
  public static <T> T select(T compBotValue, T practiceBotValue) {
    if (isCompetitionBot()) {
      return compBotValue;
    }

    if (m_bot.equals(Name.Practice)) {
      return practiceBotValue;
    }

    Logger.tag("RobotName")
        .warn("Value not found for bot {}, using comp bot value", m_bot.toString());

    return compBotValue;
  }

  /**
   * Select between competiton or other bot name values. Typically used in Constants.java
   *
   * @param <T> Generic type to select, typically set by the type of the parameters
   * @param compBotValue value to set if this is the competition bot
   * @param otherBotValues array of pairs of bot names and values for that robot.
   * @return value to set
   */
  public static <T> T select(T compBotValue, Pair<Name, T>[] otherBotValues) {
    if (isCompetitionBot()) {
      return compBotValue;
    }

    Logger.tag("RobotName").trace("Using other bot value for bot {}", m_bot.toString());

    for (var botValue : otherBotValues) {
      if (m_bot == botValue.getFirst()) {
        return botValue.getSecond();
      }
    }

    Logger.tag("RobotName")
        .warn("Value not found for bot {}, using comp bot value", m_bot.toString());

    return compBotValue;
  }

  /**
   * Check if this system has a certain mac address in any network device. Taken from
   * https://github.dev/Team254/FRC-2023-Public Constants.java
   *
   * @param mac_address Mac address to check.
   * @return true if some device with this mac address exists on this system.
   */
  private static boolean hasMacAddress(final String mac_address) {
    try {
      Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
      while (nwInterface.hasMoreElements()) {
        NetworkInterface nis = nwInterface.nextElement();
        if (nis == null) {
          continue;
        }
        StringBuilder device_mac_sb = new StringBuilder();
        Logger.tag("RobotName").info("hasMacAddress: NIS: {}", nis.getDisplayName());
        byte[] mac = nis.getHardwareAddress();
        if (mac != null) {
          for (int i = 0; i < mac.length; i++) {
            device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
          }
          String device_mac = device_mac_sb.toString();
          Logger.tag("RobotName")
              .info("hasMacAddress: NIS {}, device_mac: ", nis.getDisplayName(), device_mac);
          if (mac_address.equals(device_mac)) {
            Logger.tag("RobotName").info("hasMacAddress: ** Mac address match! ", device_mac);
            return true;
          }
        } else {
          Logger.tag("RobotName").warn("hasMacAddress: Address doesn't exist or is not accessible");
        }
      }

    } catch (SocketException e) {
      Logger.tag("RobotName").error(e);
    }
    return false;
  }
}
