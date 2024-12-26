package frc.lib.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleSim {
  private final FlywheelSim m_driveMotor;
  private final FlywheelSim m_turningMotor;

  /**
   * Overly simplistic model of swerve module with very simple dynamics using WPILib DCMotorSim
   * classes. The only input is a DC Motor and an inertia. Note that gearing should be factored in
   * using the withReduction method of the DCMotor class.
   *
   * @param driveMotor DCMotor including reductions for the drive motor
   * @param turningMotor DCMotor including reductions for the turning motor
   * @param driveInertia Modeled inertia of the drive motor
   * @param turningInertia Modeled inertia of the turning motor
   */
  public SwerveModuleSim(
      DCMotor driveMotor,
      double drivekV,
      double drivekA,
      DCMotor turningMotor,
      double turningkV,
      double turningkA) {
    m_driveMotor =
        new FlywheelSim(LinearSystemId.identifyVelocitySystem(drivekV, drivekA), driveMotor, 1.0);
    m_turningMotor =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(turningkV, turningkA), turningMotor, 1.0);
  }

  public double getTurningMotorVelocityRPM() {
    return m_turningMotor.getAngularVelocityRPM();
  }

  public double getDriveMotorVelocityRPM() {
    return m_driveMotor.getAngularVelocityRPM();
  }

  public void iterate(double driveVoltage, double turningVoltage, double dtSeconds) {
    m_turningMotor.setInputVoltage(turningVoltage);
    m_driveMotor.setInputVoltage(driveVoltage);
    m_driveMotor.update(dtSeconds);
    m_turningMotor.update(dtSeconds);
  }

  public double getTurningMotorCurrentDrawAmps() {
    return m_turningMotor.getCurrentDrawAmps();
  }

  public double getDriveMotorCurrentDrawAmps() {
    return m_driveMotor.getCurrentDrawAmps();
  }

  public double getCurrentDrawAmps() {
    return m_driveMotor.getCurrentDrawAmps() + m_turningMotor.getCurrentDrawAmps();
  }
}
