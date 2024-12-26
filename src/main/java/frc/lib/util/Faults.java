package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import org.tinylog.Logger;

public class Faults {
  private static HashMap<String, Fault> s_faults = new HashMap<>();
  protected static List<Consumer<Fault>> s_callbacks = new ArrayList<>();

  private Faults() {}

  public static int count() {
    int cnt = 0;
    for (var fault : s_faults.values()) {
      if (fault.m_message.length() > 0) {
        cnt++;
      }
    }
    return cnt;
  }

  /** Fatal faults not clearable */
  public static void clearAll() {
    for (var fault : s_faults.values()) {
      fault.clear();
    }
  }

  public static class Fault {
    private String m_message = "";
    private boolean m_present = false;
    private boolean m_fatal = false;
    private String m_tag = "";
    private final Timer printTimer = new Timer();
    private final double kPrintThrottleTimeSeconds = 10.0;

    private void triggerCallbacks() {
      for (var callback : Faults.s_callbacks) {
        callback.accept(this);
      }
    }

    public Fault(String tag) {
      printTimer.stop();
      printTimer.reset();
      m_tag = tag;
    }

    private void printThrottled(String message) {
      if (printTimer.hasElapsed(kPrintThrottleTimeSeconds) || printTimer.get() == 0.0) {
        Logger.tag(m_tag).error(message);
        printTimer.restart();
      }
    }

    public void fatal(String message) {
      m_present = true;
      m_fatal = true;
      m_message = message;
      printThrottled(message);
      triggerCallbacks();
    }

    public void error(String message) {
      m_present = true;
      if (!m_fatal) {
        m_message = message;
      }

      printThrottled(message);
      triggerCallbacks();
    }

    /*
     * Fatal faults not clearable
     */
    public void clear() {
      if (!m_present || m_fatal) {
        return;
      }

      m_message = "";
      m_present = false;

      triggerCallbacks();
    }

    public void markInitialized() {
      if (!m_present) {
        triggerCallbacks();
      }
    }

    public String message() {
      return m_message;
    }

    public String tag() {
      return m_tag;
    }

    public boolean present() {
      return m_present;
    }

    public boolean fatal() {
      return m_fatal;
    }
  }

  public static Fault subsystem(String subsystem) {
    if (s_faults.containsKey(subsystem)) {
      return s_faults.get(subsystem);
    }
    Fault fault = new Fault(subsystem);
    s_faults.put(subsystem, fault);
    return fault;
  }

  public static void onFault(Consumer<Fault> callback) {
    s_callbacks.add(callback);
  }
}
