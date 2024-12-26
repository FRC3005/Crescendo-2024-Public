package frc.lib.vendor.sensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class TCA9548A implements I2CMux {
  private final int kI2CAddress;
  private final I2C m_i2c;

  public TCA9548A(int addr) {
    kI2CAddress = addr;
    m_i2c = new I2C(Port.kMXP, addr);
  }

  public TCA9548A() {
    // Default address for the common PCB available
    this(0x70);
  }

  @Override
  public int enabledBuses() {
    byte[] result = new byte[1];
    m_i2c.readOnly(result, 1);
    return result[0];
  }

  @Override
  public void setEnabledBuses(int... buses) {
    int writeValue = 0;
    for (int b : buses) {
      if (b >= availableBuses() || b < 0) {
        DriverStation.reportError("Invalid bus enabled on I2C Mux: " + b, true);
      } else {
        writeValue |= 1 << b;
      }
    }
    m_i2c.write(kI2CAddress, writeValue);
  }

  @Override
  public int availableBuses() {
    return 8;
  }
}
