package frc.lib.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class LimelightSim {
  private final SimDeviceSim m_simDevice;
  private SimDouble m_targetArea;
  private SimDouble m_skew;
  private SimDouble m_latency;
  private SimDouble m_tx;
  private SimDouble m_ty;
  private SimBoolean m_valid;

  /** Create a simulated limelight. Access these values with a Limelight object. */
  public LimelightSim() {
    m_simDevice = new SimDeviceSim("Limelight");
    m_targetArea = m_simDevice.getDouble("Target Area");
    m_skew = m_simDevice.getDouble("Skew");
    m_latency = m_simDevice.getDouble("Latency");
    m_tx = m_simDevice.getDouble("Tx");
    m_ty = m_simDevice.getDouble("Ty");
    m_valid = m_simDevice.getBoolean("Valid");
  }

  /** Reset all sim values to defaults. */
  public void reset() {
    m_targetArea.set(0);
    m_skew.set(0);
    m_latency.set(0);
    m_tx.set(0);
    m_ty.set(0);
    m_valid.set(false);
  }

  /**
   * Set the target area in percent of the image
   *
   * <p>TODO: is this range [0, 1] or [0, 100]
   *
   * @param targetArea percent of the image
   */
  public void setTargetArea(double targetArea) {
    m_targetArea.set(targetArea);
  }

  /**
   * Set the target skew
   *
   * @param skew scew or rotation [-90, 0] degrees
   */
  public void setSkew(double skew) {
    m_skew.set(skew);
  }

  /**
   * Set the latency in ms. Actual value will have 11ms added.
   *
   * @param latencyMs latency in ms
   */
  public void setLatency(double latencyMs) {
    m_latency.set(latencyMs);
  }

  /**
   * Set the X offset in degrees
   *
   * @param degrees offset in degrees
   */
  public void setOffsetX(double degrees) {
    m_tx.set(degrees);
  }

  /**
   * Set the Y offset in degrees
   *
   * @param degrees offset in degrees
   */
  public void setOffsetY(double degrees) {
    m_ty.set(degrees);
  }

  /**
   * Check if the target is valid
   *
   * @param valid true if valid false if not
   */
  public void setValid(boolean valid) {
    m_valid.set(valid);
  }
}
