package frc.lib.vendor.sensor;

public interface I2CMux {
  /**
   * Read list of enabled buses from the device.
   *
   * @return bit field of enabled buses
   */
  public int enabledBuses();

  /**
   * Set the list of enabled buses
   *
   * @param buses list of buses to enable
   */
  public void setEnabledBuses(int... buses);

  /**
   * Number of available buses
   *
   * @return number of available buses
   */
  public int availableBuses();
}
