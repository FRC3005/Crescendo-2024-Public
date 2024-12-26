package frc.lib.sim;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveSim {
  private final SwerveDriveKinematics m_kinematics;

  public SwerveDriveSim(SwerveDriveKinematics kinematics) {
    m_kinematics = kinematics;
  }

  public void iterate(SwerveModuleState[] moduleStates) {
    var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
  }
}
