package frc.lib.controller;

import edu.wpi.first.math.controller.PIDController;
import java.util.ArrayList;

public class GainScheduledPIDController extends PIDController {
  private class GainSet {
    public final PIDGains gains;
    public final double errorThreshold;

    public GainSet(PIDGains gains, double errorThreshold) {
      this.gains = gains;
      this.errorThreshold = errorThreshold;
    }
  }

  private final ArrayList<GainSet> m_gains = new ArrayList<>();
  private final PIDGains m_defaultGains;

  public GainScheduledPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);
    m_defaultGains = new PIDGains(kp, ki, kd);
  }

  public GainScheduledPIDController(double kp, double ki, double kd, double period) {
    super(kp, ki, kd, period);
    m_defaultGains = new PIDGains(kp, ki, kd);
  }

  public void addScheduledGain(double kp, double ki, double kd, double errorThreshold) {
    m_gains.add(new GainSet(new PIDGains(kp, ki, kd), errorThreshold));
  }

  private void updateGains(double error) {
    for (var gain : m_gains) {
      if (Math.abs(error) < gain.errorThreshold) {
        this.setPID(gain.gains.P, gain.gains.I, gain.gains.D);
        return;
      }
    }
    this.setPID(m_defaultGains.P, m_defaultGains.I, m_defaultGains.D);
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    updateGains(this.getPositionError());
    return super.calculate(measurement, setpoint);
  }

  @Override
  public double calculate(double measurement) {
    updateGains(this.getPositionError());
    return super.calculate(measurement);
  }
}
