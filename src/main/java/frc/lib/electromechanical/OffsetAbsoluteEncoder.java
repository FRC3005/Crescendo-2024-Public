package frc.lib.electromechanical;

import edu.wpi.first.math.MathUtil;

public class OffsetAbsoluteEncoder implements AbsoluteEncoder {
  private final double m_offset;
  private final double m_min;
  private final double m_max;
  private final AbsoluteEncoder m_encoder;
  private static final double kEpsilon = 1E-9;

  public OffsetAbsoluteEncoder(AbsoluteEncoder encoder, double offset, double min, double max) {
    m_encoder = encoder;
    m_offset = offset;
    m_min = min;
    m_max = max;
  }

  @Override
  public double getPosition() {
    double result = m_encoder.getPosition() - m_offset;
    result = MathUtil.inputModulus(result, m_min, m_max);

    // Treat min and max as the same point, so range of [m_min, m_max)
    // TODO: (Is this the right approach?)
    if (result == m_max) {
      result = m_min;
    }
    return result;
  }

  public AbsoluteEncoder getInner() {
    return m_encoder;
  }
}
