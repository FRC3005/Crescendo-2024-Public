package frc.robot.indicators;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.Faults;
import frc.lib.util.Faults.Fault;
import java.util.HashMap;

/**
 * Some variation of this https://www.adafruit.com/product/1426 This implentation is not quite right
 * yet. The 'Faults' class is not right. Having a generic class allows 'library' code to call into
 * faults as well. Right now that is used for Spark initialization failures. However that should
 * just pass back into the caller to then set the LED instead.
 */
public class StatusLedStrip8 {
  private static StatusLedStrip8 instance = null;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(8);

  private final Color8Bit kGood = new Color8Bit(0, 25, 0);
  private final Color8Bit kFatal = new Color8Bit(25, 0, 0);
  private final Color8Bit kWarning = new Color8Bit(25, 25, 0);
  private static final Color8Bit kBlue = new Color8Bit(0, 0, 25);
  private static final Color8Bit kBlack = new Color8Bit(0, 0, 0);

  private final HashMap<String, String> m_systemStatusStrings = new HashMap<>();

  public StatusLedStrip8(int pwmChannel) {
    m_led = new AddressableLED(pwmChannel);
    m_led.setLength(8);
    m_led.setData(m_buffer);
    m_led.start();

    m_buffer.setLED(0, kGood);

    Faults.onFault(this::faultHandler);
    String[] none = new String[1];
    none[0] = "None";
    SmartDashboard.putStringArray("Faults", none);
    instance = this;
  }

  public static void setHasTagInView(boolean tagInView) {
    if (instance != null) {
      instance.m_buffer.setLED(5, tagInView ? kBlue : kBlack);
      instance.m_led.setData(instance.m_buffer);
    }
  }

  private void setTag(String tag, Color8Bit color) {
    int idx = 0;
    switch (tag.toLowerCase()) {
      case "arm":
        idx = 1;
        break;
      case "robot state":
        idx = 2;
        break;
      case "drive":
        idx = 3;
        break;
      case "launcher":
        idx = 4;
        break;
        // case "diverter":
        //   idx = 5;
        //   break;
      case "intake":
        idx = 6;
        break;
      case "climber":
        idx = 7;
        break;
      case "sparkmax":
      default:
        idx = 0;
        break;
    }
    m_buffer.setLED(idx, color);
  }

  private void faultHandler(Fault fault) {
    String tag = fault.tag();
    Color8Bit color = fault.fatal() ? kFatal : fault.present() ? kWarning : kGood;

    setTag(tag, color);

    m_led.setData(m_buffer);

    // TODO: This should just go in the Faults.java
    if (fault.present()) {
      m_systemStatusStrings.put(fault.tag(), fault.tag() + ": " + fault.message());
      SmartDashboard.putStringArray(
          "Faults", m_systemStatusStrings.values().toArray(new String[0]));
    }
  }
}
