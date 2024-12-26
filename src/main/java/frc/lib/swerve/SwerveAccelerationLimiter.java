package frc.lib.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveAccelerationLimiter {
  public SwerveAccelerationLimiter(double translationalLimit) {
    setAccelerationLimit(translationalLimit);
  }

  /**
   * Take in a chassis speed and output an acceleration limited output. This is stateful and looks
   * at previous and next state.
   *
   * @param speeds desired chassis speeds
   * @param dt dt of the loop calling this function
   * @return acceleration limited chassis speeds
   */
  public ChassisSpeeds transform(ChassisSpeeds speeds) {
    // m_prevChassisSpeeds of null means state is reset
    if (m_prevChassisSpeeds == null) {
      m_prevChassisSpeeds = speeds;
      m_rateLimitVx.reset(speeds.vxMetersPerSecond);
      m_rateLimitVy.reset(speeds.vyMetersPerSecond);
      return speeds;
    }

    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            m_rateLimitVx.calculate(speeds.vxMetersPerSecond),
            m_rateLimitVy.calculate(speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond);

    m_prevChassisSpeeds = updatedSpeeds;
    return updatedSpeeds;
  }

  /**
   * Reset the state such that the time time transform is called, its speed is the current state.
   * Use this before or after enabling this mode.
   */
  public void reset() {
    m_prevChassisSpeeds = null;
  }

  public void setAccelerationLimit(double translationalLimit) {
    m_prevChassisSpeeds = null;
    m_rateLimitVx = new SlewRateLimiter(translationalLimit);
    m_rateLimitVy = new SlewRateLimiter(translationalLimit);
  }

  private ChassisSpeeds m_prevChassisSpeeds = null;
  private SlewRateLimiter m_rateLimitVx;
  private SlewRateLimiter m_rateLimitVy;
}
