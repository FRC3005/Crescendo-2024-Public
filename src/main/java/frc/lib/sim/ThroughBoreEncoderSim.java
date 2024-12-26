package frc.lib.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.lib.vendor.sensor.ThroughBoreEncoder;

public class ThroughBoreEncoderSim {
  private final ThroughBoreEncoder m_encoder;
  private final DutyCycleSim m_dutyCycleSim;

  private final SimDouble m_simPosition;
  private final SimDouble m_simDistancePerRotation;

  public ThroughBoreEncoderSim(ThroughBoreEncoder encoder) {
    m_encoder = encoder;
    m_dutyCycleSim = DutyCycleSim.createForChannel(encoder.getDigitalSource().getChannel());
    SimDeviceSim wrappedSimDevice =
        new SimDeviceSim(
            "DutyCycle:DutyCycleEncoder" + "[" + encoder.getDigitalSource().getChannel() + "]");
    m_simPosition = wrappedSimDevice.getDouble("position");
    m_simDistancePerRotation = wrappedSimDevice.getDouble("distance_per_rot");
    m_dutyCycleSim.setFrequency(1000);
  }

  public void setAbsolutePosition(double position) {
    double scalar = m_encoder.getPositionScalar();
    double offset = m_encoder.getPositionOffset();
    boolean inverted = m_encoder.getPositionInverted();
    double dutyCycle = 0.0;

    if (inverted) {
      dutyCycle = ((position + offset) / scalar) - 1.0;
    } else {
      dutyCycle = (position + offset) / scalar;
    }
    dutyCycle = MathUtil.inputModulus(dutyCycle, 0, 1.0);

    m_dutyCycleSim.setOutput(dutyCycle);
  }

  public void setDutyCycle(double dutyCycle) {
    m_dutyCycleSim.setOutput(dutyCycle);
  }

  public void setRelativePosition(double position) {
    m_simPosition.set(position);
  }

  public void setConnectionState(boolean connected) {
    if (connected) {
      m_dutyCycleSim.setFrequency(1000);
    } else {
      m_dutyCycleSim.setFrequency(0);
    }
  }
}
