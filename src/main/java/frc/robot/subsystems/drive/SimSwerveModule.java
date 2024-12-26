package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.controller.SimpleMotorFeedforward;
import frc.lib.electromechanical.Encoder;
import frc.lib.electromechanical.EncoderSupplier;
import frc.lib.swerve.SwerveModule;
import frc.lib.telemetry.TelemetryBuilder;
import frc.robot.constants.DrivetrainConstants;
import org.tinylog.Logger;

/** Add your docs here. */
public class SimSwerveModule implements SwerveModule {
  private SwerveModuleState m_desiredState = new SwerveModuleState();
  private SimpleMotorFeedforward m_driveFeedforward =
      new SimpleMotorFeedforward(DrivetrainConstants.kDriveFeedforward);

  private final DCMotorSim m_driveMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              m_driveFeedforward.kv * 0.0100, m_driveFeedforward.ka * 0.0100),
          DrivetrainConstants.kDriveMotorSim,
          1.0 / DrivetrainConstants.kDriveMotorReduction);
  private final DCMotorSim m_turnMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DrivetrainConstants.kTurningFeedforward.kv * 0.0100,
              DrivetrainConstants.kTurningFeedforward.ka * 0.0100),
          DrivetrainConstants.kTurningMotorSim,
          1.0 / DrivetrainConstants.kTurningModuleGearRatio);

  PIDController m_drivePIDController =
      new PIDController(
          DrivetrainConstants.kDriveMotorPIDGains.P,
          DrivetrainConstants.kDriveMotorPIDGains.I,
          DrivetrainConstants.kDriveMotorPIDGains.D);
  PIDController m_turningPIDController =
      new PIDController(
          DrivetrainConstants.kTurningMotorPIDGains.P,
          DrivetrainConstants.kTurningMotorPIDGains.I,
          DrivetrainConstants.kTurningMotorPIDGains.D);

  public SimSwerveModule() {
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotorSim.getAngularVelocityRPM() * DrivetrainConstants.kDriveEncoderVelocityFactor,
        Rotation2d.fromRadians(
            m_turnMotorSim.getAngularPositionRotations()
                * DrivetrainConstants.kTurningEncoderPositionFactor));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation =
        Rotation2d.fromRadians(
            m_turnMotorSim.getAngularPositionRotations()
                * DrivetrainConstants.kTurningEncoderPositionFactor);

    // Optimize the reference state to avoid spinning further than 90 degrees
    m_desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);
    m_voltageDriveMode = false;
  }

  @Override
  public void runSetpoint(SwerveModuleState state) {
    m_voltageDriveMode = false;
    m_desiredState = state;
  }

  @Override
  public void periodic() {
    var encoderRotation =
        Rotation2d.fromRadians(
            m_turnMotorSim.getAngularPositionRotations()
                * DrivetrainConstants.kTurningEncoderPositionFactor);

    var desiredState = m_desiredState;

    if (RobotState.isDisabled()) {
      desiredState = new SwerveModuleState();
    }

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(
            m_driveMotorSim.getAngularVelocityRPM()
                * DrivetrainConstants.kDriveEncoderVelocityFactor,
            desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
            encoderRotation.getRadians(), m_desiredState.angle.getRadians());

    // final double turnFeedforward =
    //     Drivetrain.kTurningFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    if (m_voltageDriveMode) {
      m_driveMotorSim.setInputVoltage(m_voltageDrive);
    } else {
      m_driveMotorSim.setInputVoltage(driveOutput + driveFeedforward);
    }
    m_turnMotorSim.setInputVoltage(turnOutput);

    m_driveMotorSim.update(0.02);
    m_turnMotorSim.update(0.02);
  }

  @Override
  public void holdHeading(double speedMetersPerSecond) {
    setDesiredState(new SwerveModuleState(speedMetersPerSecond, m_desiredState.angle));
    m_voltageDriveMode = false;
  }

  @Override
  public void resetEncoders() {
    m_driveMotorSim.setState(0, 0);
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotorSim.getAngularPositionRotations()
            * DrivetrainConstants.kDriveEncoderPositionFactor,
        new Rotation2d(
            m_turnMotorSim.getAngularPositionRotations()
                * DrivetrainConstants.kTurningEncoderPositionFactor));
  }

  private boolean m_voltageDriveMode = false;
  private double m_voltageDrive = 0.0;

  @Override
  public void setVoltageAngle(double voltage, Rotation2d angle) {
    m_desiredState.speedMetersPerSecond = 0.0;
    m_desiredState.angle = angle;
    m_voltageDriveMode = true;
    m_voltageDrive = voltage;
  }

  @Override
  public SwerveModuleState getActualState() {
    return getState();
  }

  private Encoder dcMotorSimWrapper(DCMotorSim sim, double velocityFactor, double positionFactor) {
    return new EncoderSupplier(
        () -> sim.getAngularVelocityRPM() * velocityFactor,
        () -> sim.getAngularPositionRotations() * positionFactor,
        (val) -> sim.setState(val, sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindSendableChild("Drive Controller", m_drivePIDController);
    builder.bindSendableChild("Turn Controller", m_turningPIDController);
    builder.addDoubleProperty(
        "Velocity Setpoint",
        () -> m_desiredState.speedMetersPerSecond,
        val -> setDesiredState(new SwerveModuleState(val, m_desiredState.angle)));
    builder.addDoubleProperty(
        "Turning Setpoint",
        () -> m_desiredState.angle.getRadians(),
        (val) ->
            setDesiredState(
                new SwerveModuleState(m_desiredState.speedMetersPerSecond, new Rotation2d(val))));

    builder.addDoubleProperty(
        "Drive kS", () -> m_driveFeedforward.ks, (k) -> m_driveFeedforward.ks = k);
    builder.addDoubleProperty(
        "Drive kA", () -> m_driveFeedforward.ka, (k) -> m_driveFeedforward.ka = k);
    builder.addDoubleProperty(
        "Drive kV", () -> m_driveFeedforward.kv, (k) -> m_driveFeedforward.kv = k);

    builder.bindChild(
        "TurnEncoder",
        dcMotorSimWrapper(
            m_turnMotorSim,
            DrivetrainConstants.kTurningEncoderVelocityFactor,
            DrivetrainConstants.kTurningEncoderPositionFactor));
    builder.bindChild(
        "DriveEncoder",
        dcMotorSimWrapper(
            m_driveMotorSim,
            DrivetrainConstants.kDriveEncoderVelocityFactor,
            DrivetrainConstants.kDriveEncoderPositionFactor));

    builder.addDoubleProperty("Drive Current", () -> m_driveMotorSim.getCurrentDrawAmps(), null);
    builder.addDoubleProperty("Turning Current", () -> m_turnMotorSim.getCurrentDrawAmps(), null);
  }

  @Override
  public void setDriveCurrent(int currentAmps) {
    Logger.tag("SimSwerve").info("Setting drive current limit to {}", currentAmps);
  }
}
