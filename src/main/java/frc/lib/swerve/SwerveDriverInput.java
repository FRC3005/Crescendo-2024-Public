package frc.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriverInput {
  private final SwerveDrive m_drive;

  public SwerveDriverInput(SwerveDrive drive) {
    m_drive = drive;
  }

  public ChassisSpeeds drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ySpeed = ySpeed * m_drive.getMaxSpeed();
    xSpeed = xSpeed * m_drive.getMaxSpeed();
    rot = rot * m_drive.getMaxAngularVelocity();

    return fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_drive.getHeading())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
  }
}
