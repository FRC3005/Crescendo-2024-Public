package frc.lib.vendor.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.electromechanical.AbsoluteEncoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.robot.Robot;

public class ThroughBoreEncoder implements AbsoluteEncoder {
  private final DigitalInput m_digitalSource;
  private final DutyCycle m_dutyCycle;
  private final DutyCycleEncoder m_encoder;
  private double m_offset = 0.0;
  private final double m_scalar;
  private final boolean m_inverted;

  public ThroughBoreEncoder(int channel, double offset, double scalar, boolean invert) {
    m_digitalSource = new DigitalInput(channel);
    m_dutyCycle = new DutyCycle(m_digitalSource);
    m_encoder = new DutyCycleEncoder(m_dutyCycle);
    m_offset = offset;
    m_scalar = scalar;
    m_inverted = invert;
  }

  public ThroughBoreEncoder(int channel) {
    this(channel, 0.0, 1.0, false);
  }

  public DigitalSource getDigitalSource() {
    return m_digitalSource;
  }

  @Override
  public double getPosition() {
    double result;
    if (m_inverted) {
      result = ((1.0 - m_dutyCycle.getOutput()) * m_scalar - m_offset);
    } else {
      result = (m_dutyCycle.getOutput() * m_scalar - m_offset);
    }
    // return MathUtil.inputModulus(result, 0, m_scalar);
    return result;
  }

  @Override
  public boolean isConnected() {
    return m_encoder.isConnected();
  }

  public double dutyCycle() {
    if (m_inverted) {
      return 1.0 - m_dutyCycle.getOutput();
    }
    return m_dutyCycle.getOutput();
  }

  public double getFrequency() {
    return m_dutyCycle.getFrequency();
  }

  public double getRelativePosition() {
    return m_encoder.get();
  }

  public void setPositionOffset(double offset) {
    m_offset = offset;
  }

  public double getPositionOffset() {
    return m_offset;
  }

  public double getPositionScalar() {
    return m_scalar;
  }

  public boolean getPositionInverted() {
    return m_inverted;
  }

  /**
   * There is a quirk when first starting code that the duty cycle class reads back a frequency that
   * is too low, and all the measurements are off. This function will block until the correct range
   * is found.
   *
   * @return true if range is found
   */
  @Override
  public boolean waitForInit(double timeoutSeconds) {
    // Values from data sheet plus some margin
    final double expectedMinFrequency = 924.0;
    final double expectedMaxFrequency = 1024.0;
    var initTimer = new Timer();
    initTimer.start();

    if (Robot.isSimulation()) {
      return true;
    }

    while (!initTimer.hasElapsed(timeoutSeconds)) {
      // Through bore encoder frequency should be in a specific range, when
      // outside that range, we know its not working quite right.
      double freq = getFrequency();
      if (isConnected() && freq > expectedMinFrequency && freq < expectedMaxFrequency) {
        return true;
      }

      Timer.delay(0.05);
    }

    return false;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    m_encoder.initSendable(builder);
    builder.bindSendableChild("Digital Source", m_digitalSource);
    builder.addDoubleProperty("Absolute Position", () -> getPosition(), null);
    builder.addDoubleProperty("Raw Sensor Output", this::dutyCycle, null);
    builder.addDoubleProperty("Offset", () -> m_offset, null);
    builder.addDoubleProperty("Scalar", () -> m_scalar, null);
    builder.addBooleanProperty("Inverted", () -> m_inverted, null);
  }
}
