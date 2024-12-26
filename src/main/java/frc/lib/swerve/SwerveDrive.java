// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.telemetry.DataTypes;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.Robot;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import org.tinylog.Logger;

public abstract class SwerveDrive extends SubsystemBase implements TelemetryNode {
  public enum ModuleLocation {
    frontLeft(0),
    frontRight(1),
    rearLeft(2),
    rearRight(3);

    public final int value;
    private static final ModuleLocation[] m_mapping =
        new ModuleLocation[] {frontLeft, frontRight, rearLeft, rearRight};

    private ModuleLocation(int v) {
      this.value = v;
    }

    public static ModuleLocation fromInt(int v) {
      return m_mapping[v];
    }
  }

  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;

  // Sim only object
  private SwerveDriveWheelPositions m_previousModulePositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
          });

  // The gyro sensor
  private final ADIS16470 m_gyro;

  private final SwerveDriveKinematics m_kinematics;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  SwerveDriveOdometry m_carpetOdometry;

  private final Field2d m_field = new Field2d();
  private final Field2d m_field2 = new Field2d();

  private double m_maxSpeed;
  private double m_maxAngularVelocity;

  private final SwerveSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint m_currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });

  /**
   * Create a Swerve Drive module
   *
   * @param frontLeft Swerve Module
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve drive kinematics
   * @param gyro used for odometry and field centric driving
   * @param maxSpeed of the wheels used to normalize wheel speeds
   */
  public SwerveDrive(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight,
      SwerveDriveKinematics kinematics,
      ADIS16470 gyro,
      double maxSpeed,
      double maxAngularVelocity) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    m_gyro = gyro;
    m_kinematics = kinematics;
    m_maxSpeed = maxSpeed;
    m_maxAngularVelocity = maxAngularVelocity;
    m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), getModulePositions());
    m_carpetOdometry =
        new SwerveDriveOdometry(
            DrivetrainConstants.kDriveCarpetKinematics,
            m_gyro.getRotation2d(),
            getModulePositions());

    m_setpointGenerator =
        new SwerveSetpointGenerator(kinematics, DrivetrainConstants.kModuleLocations);
  }

  private ChassisSpeeds m_chassisSpeed = new ChassisSpeeds();
  private ChassisSpeeds m_desiredSetpoint = new ChassisSpeeds();

  private double m_cor_x = 0.0;
  private double m_cor_y = 0.0;

  @Override
  public void periodic() {
    // Run swerve modules if they require it
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    m_chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    SwerveModulePosition[] module_positions =
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
        };

    if (Robot.isSimulation()) {
      var modulePositions = new SwerveDriveWheelPositions(module_positions);
      double dtheta = m_kinematics.toTwist2d(m_previousModulePositions, modulePositions).dtheta;
      m_gyro.setAngle(Units.radiansToDegrees(dtheta + m_gyro.getRotation2d().getRadians()));
      m_previousModulePositions = modulePositions;
    }

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), module_positions);
    m_carpetOdometry.update(m_gyro.getRotation2d(), module_positions);
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field.getObject("carpet odometry").setPose(m_carpetOdometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
  }

  /**
   * Return the current velocity of the chassis as a ChassisSpeeds object.
   *
   * @return velocity of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeed;
  }

  /**
   * Predict the motion between the current position and a future state.
   *
   * @param lookahead Time in seconds to predict ahead.
   * @return twist2d represnting the change in pose over the lookahead time.
   */
  public Twist2d getPredictedMotion(double lookahead) {
    ChassisSpeeds chassisSpeed = getChassisSpeeds();
    return new Twist2d(
        chassisSpeed.vxMetersPerSecond * lookahead,
        chassisSpeed.vyMetersPerSecond * lookahead,
        0.0);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    m_carpetOdometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot.
   *
   * @param chassisSpeeds the object holding the target module drive states
   */
  public void driveOld(ChassisSpeeds chassisSpeeds) {
    // If nothing is commanded, hold the same position
    if (Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.001
        && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.001
        && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.001) {
      m_desiredSetpoint = new ChassisSpeeds();
      holdAllModulesRotation(0.0);
      return;
    }

    m_desiredSetpoint = chassisSpeeds;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, 0.02), new Translation2d(m_cor_x, m_cor_y));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot.
   *
   * @param chassisSpeeds the object holding the target module drive states
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    // If nothing is commanded, hold the same position
    if (Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.001
        && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.001
        && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.001) {
      m_desiredSetpoint = new ChassisSpeeds();
      holdAllModulesRotation(0.0);
      return;
    }

    // TODO: Do testing on real robot without discretize call
    m_desiredSetpoint = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    // TODO: Should actual setpoint generation happen in periodic instead?
    m_currentSetpoint =
        m_setpointGenerator.generateSetpoint(
            DrivetrainConstants.kModuleLimits, m_currentSetpoint, m_desiredSetpoint, 0.02);

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < DrivetrainConstants.kModuleLocations.length; i++) {
      // Optimize setpoints
      swerveModuleStates[i] =
          SwerveModuleState.optimize(
              m_currentSetpoint.moduleStates()[i],
              DrivetrainConstants.kModuleLocations[i].getAngle());
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ySpeed = ySpeed * m_maxSpeed;
    xSpeed = xSpeed * m_maxSpeed;
    rot = rot * m_maxAngularVelocity;

    drive(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  /**
   * Drive the robot with all modules pointing in the same direction, with the magnitude set by the
   * voltage. Useful for characterization.
   *
   * @param voltage sent directly to all drive motors
   * @param direction robot relative angle to drive the modules in
   */
  private void driveVoltageForwardTest(double voltage) {
    var direction = new Rotation2d();
    m_frontLeft.setVoltageAngle(voltage, direction);
    m_frontRight.setVoltageAngle(voltage, direction);
    m_rearLeft.setVoltageAngle(voltage, direction);
    m_rearRight.setVoltageAngle(voltage, direction);
  }

  private void driveVoltageRotateTest(double voltage) {
    m_frontLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(-45.0));
    m_frontRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(45.0));
    m_rearLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(45.0));
    m_rearRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(-45.0));
  }

  private void holdAllModulesRotation(double speedMetersPerSecond) {
    m_frontLeft.holdHeading(speedMetersPerSecond);
    m_frontRight.holdHeading(speedMetersPerSecond);
    m_rearLeft.holdHeading(speedMetersPerSecond);
    m_rearRight.holdHeading(speedMetersPerSecond);
  }

  /** Set the swerve drive into an X which is not drivable but should help prevent being pushed. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set an individual module state independently of all other modules. This should only be used for
   * testing/tuning.
   */
  public void testPeriodic() {
    m_frontLeft.testPeriodic();
    m_frontRight.testPeriodic();
    m_rearLeft.testPeriodic();
    m_rearRight.testPeriodic();
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  public SwerveModuleState[] getActualModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getActualState(),
      m_frontRight.getActualState(),
      m_rearLeft.getActualState(),
      m_rearRight.getActualState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    setPose(new Pose2d(m_odometry.getPoseMeters().getTranslation(), new Rotation2d()));
  }

  public void setHeading(Rotation2d rotation) {
    Logger.tag("Swerve Drive").trace("Setting heading to {} degrees", rotation.getDegrees());
    setPose(new Pose2d(m_odometry.getPoseMeters().getTranslation(), rotation));
    m_gyro.setAngle(rotation.getDegrees());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public double getMaxSpeed() {
    return m_maxSpeed;
  }

  public double getMaxAngularVelocity() {
    return m_maxAngularVelocity;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition(),
    };
  }

  public SysIdRoutine sysIdDrive() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)), null, this));
  }

  public SysIdRoutine sysIdRotation(double duration) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Seconds.of(1)), Volts.of(5), Seconds.of(duration)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)), null, this));
  }

  public WheelDiameterChar wheelDiameterCharacterization() {
    return new WheelDiameterChar(
        this,
        () -> {
          SwerveModulePosition[] positions = getModulePositions();
          double[] result = new double[positions.length];
          for (int i = 0; i < result.length; i++) {
            result[i] = // positions[i].distanceMeters;
                (2.0 * positions[i].distanceMeters / DrivetrainConstants.kWheelDiameterMeters);
          }
          return result;
        },
        () -> getHeading().getRadians(),
        DrivetrainConstants.kModuleLocations[0].getNorm(),
        10.0);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity vx", () -> m_chassisSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Velocity vy", () -> m_chassisSpeed.vyMetersPerSecond, null);
    builder.addDoubleProperty("Velocity omega", () -> m_chassisSpeed.omegaRadiansPerSecond, null);
    builder.addDoubleProperty(
        "Desired Setpoint vx", () -> m_desiredSetpoint.vxMetersPerSecond, null);
    builder.addDoubleProperty(
        "Desired Setpoing vy", () -> m_desiredSetpoint.vyMetersPerSecond, null);
    builder.addDoubleProperty(
        "Desired Velocity omega", () -> m_desiredSetpoint.omegaRadiansPerSecond, null);
    builder.addDoubleProperty(
        "Setpoint vx", () -> m_currentSetpoint.chassisSpeeds().vxMetersPerSecond, null);
    builder.addDoubleProperty(
        "Setpoing vy", () -> m_currentSetpoint.chassisSpeeds().vyMetersPerSecond, null);
    builder.addDoubleProperty(
        "Setpoint velocity omega",
        () -> m_currentSetpoint.chassisSpeeds().omegaRadiansPerSecond,
        null);
    builder.addDoubleProperty(
        "Setpoing omega", () -> m_desiredSetpoint.omegaRadiansPerSecond, null);
    builder.bindChild("Gyro", m_gyro);
    builder.bindChild("frontLeft", m_frontLeft);
    builder.bindChild("frontRight", m_frontRight);
    builder.bindChild("rearLeft", m_rearLeft);
    builder.bindChild("rearRight", m_rearRight);
    builder.bindSendableChild("Field 2d", m_field);
    builder.addDoubleArrayProperty(
        "Swerve State", () -> DataTypes.swerveStatesToData(getModuleStates()), null);

    if (!RobotConstants.kReducedTelemetryMode) {
      builder.addDoubleProperty(
          "angular speed rads per sec",
          this::getMaxAngularVelocity,
          (val) -> m_maxAngularVelocity = val);
      builder.addDoubleProperty(
          "max speed meters per sec", () -> m_maxSpeed, (val) -> m_maxSpeed = val);
      builder.bindSendableChild("Field 2d Carpet", m_field2);
      builder.addDoubleArrayProperty(
          "Swerve SM State", () -> DataTypes.swerveStatesToData(getActualModuleStates()), null);
    }
  }
}
