package frc.lib.monitor;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public interface IsConnected {

  /**
   * Return true if the device is connected. Some components may not have this capability, so a
   * default of returning true is used.
   *
   * @return true if the device is connected.
   */
  public default boolean isConnected() {
    return true;
  }

  /**
   * Wait until isConnected() returns true, blocking the thread until the condition is met or until
   * timeoutSeconds passes. Always return true right away in simulation.
   *
   * @param timeoutSeconds max time to wait for connection
   * @return true if connected, false otherwise
   */
  public default boolean waitForInit(double timeoutSeconds) {
    if (Robot.isSimulation()) {
      return true;
    }

    Timer initTimer = new Timer();
    initTimer.start();

    while (!isConnected() && !initTimer.hasElapsed(timeoutSeconds)) {
      Timer.delay(0.05);
    }

    return isConnected();
  }
}
