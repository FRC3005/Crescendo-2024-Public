package frc.lib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controller.PIDGains;
import java.util.function.Supplier;

public class SwerveHeadingController implements Sendable {
  private final SwerveDrive m_drive;
  private final PIDController m_controller;
  private final Supplier<Rotation2d> m_headingSupplier;
  private final double m_translationScalar;
  private PIDGains m_defaultGains;

  public SwerveHeadingController(
      SwerveDrive drive, PIDController controller, Supplier<Rotation2d> headingSupplier) {
    this(drive, controller, 0.5, headingSupplier);
  }

  public SwerveHeadingController(
      SwerveDrive drive,
      PIDController controller,
      double translationScalar,
      Supplier<Rotation2d> headingSupplier) {
    m_drive = drive;
    m_controller = controller;
    m_headingSupplier = headingSupplier;
    m_translationScalar = translationScalar;
    m_defaultGains = new PIDGains(controller.getP(), controller.getI(), controller.getD());
    // TODO: These values (either constants or passed in)
    controller.setTolerance(Units.degreesToRadians(0.8));

    m_controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds transform(ChassisSpeeds speeds) {
    // if (Math.abs(m_controller.getPositionError()) < Units.degreesToRadians(2.0)) {
    //  m_controller.setPID(0.01, 0.0, 0.0);
    // } else {
    //   m_controller.setPID(m_defaultGains.P, m_defaultGains.I, m_defaultGains.D);
    // }

    SmartDashboard.putNumber("Heading Control P", m_controller.getP());
    SmartDashboard.putNumber("Heading Control Error", m_controller.getPositionError());
    speeds.omegaRadiansPerSecond =
        m_controller.calculate(
            m_drive.getHeading().getRadians(), m_headingSupplier.get().getRadians());

    if (m_controller.atSetpoint()) {
      speeds.omegaRadiansPerSecond = 0.0;
    }

    if (speeds.omegaRadiansPerSecond < -m_drive.getMaxAngularVelocity()) {
      speeds.omegaRadiansPerSecond = -m_drive.getMaxAngularVelocity();
    } else if (speeds.omegaRadiansPerSecond > m_drive.getMaxAngularVelocity()) {
      speeds.omegaRadiansPerSecond = m_drive.getMaxAngularVelocity();
    }
    speeds.vxMetersPerSecond *= m_translationScalar;
    speeds.vyMetersPerSecond *= m_translationScalar;
    return speeds;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Error", m_controller::getPositionError, null);
    builder.addDoubleProperty("Current P Gain", m_controller::getP, null);
    // builder.bindChild("Default Gians", m_defaultGains);
  }
}
