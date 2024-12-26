package frc.lib.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class ColorSensorV3Sim {
  private final SimDouble R;
  private final SimDouble G;
  private final SimDouble B;
  private final SimDouble IR;
  private final SimDouble Proximity;

  public ColorSensorV3Sim(Port port) {
    // "REV Color Sensor V3", port.value, kAddress);
    SimDeviceSim wrappedDevice = new SimDeviceSim("REV Color Sensor V3", port.value, 0x52);
    R = wrappedDevice.getDouble("Red");
    G = wrappedDevice.getDouble("Green");
    B = wrappedDevice.getDouble("Blue");
    IR = wrappedDevice.getDouble("IR");
    Proximity = wrappedDevice.getDouble("Proximity");
  }

  public void setColor(double r, double g, double b, double ir) {
    R.set(r);
    G.set(g);
    B.set(b);
    IR.set(ir);
  }

  public void setProximity(double value) {
    Proximity.set(value);
  }
}
