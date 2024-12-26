package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkBaseHandle;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkSim1;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkDutyCycleSensorSim;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controller.SimpleMotorFeedforward;
import frc.lib.electromechanical.Encoder;
import frc.lib.sim.SwerveModuleSim;
import frc.lib.swerve.SwerveModule;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.vendor.motorcontroller.SparkController;
import frc.lib.vendor.motorcontroller.SparkDutyCycleSensor;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;

public class REVSwerveModule implements SwerveModule, AutoCloseable {
  private final SparkFlex m_driveMotor;
  private final SparkFlex m_turningMotor;
  private final SparkController m_turningController;
  private final SparkController m_driveController;

  private final Encoder m_turningNeoEncoder;
  private final SparkDutyCycleSensor m_turningEncoder;
  private final Encoder m_driveNeoEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final double m_turningOffset;

  // Simulation Functions
  private final CANSparkSim1 m_turningMotorSim;
  private final CANSparkSim1 m_driveMotorSim;
  private final SwerveModuleSim m_swerveSim;
  protected final SparkDutyCycleSensorSim m_turningSensorSim;
  private SimpleMotorFeedforward m_turningFeedforward =
      new SimpleMotorFeedforward(DrivetrainConstants.kTurningFeedforward);

  private boolean m_testEnable = false;

  private boolean m_voltageDriveMode = false;
  private double m_voltageDrive = 0.0;

  // Desired state of the Spark Max itself after accounting for offset
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Create a REV Swerve Module. This module expects a REV Swerve, with a NEO drive and NEO 500 for
   * turning, with a through bore encoder, plugged into the top of the Spark Max. Each module has
   * its offset stored in flash from the hardware client.
   *
   * @param driveCanId CAN Id for the drive Spark Max
   * @param turningCanId CAN Id for the turning Spark Max, with through bore encoder
   * @param turningOffset Offset from 0 in radians which the module is set for. For a typical 4
   *     corner swerve in the range [-pi, pi] this will be one of 0, pi/2, pi, -pi/2.
   */
  public REVSwerveModule(int driveCanId, int turningCanId, double turningOffset) {
    m_driveMotor =
        new SparkFlex(driveCanId, MotorType.kBrushless)
            .withInitializer(REVSwerveModule::driveMotorConfig);
    m_turningMotor =
        new SparkFlex(turningCanId, MotorType.kBrushless)
            .withInitializer(REVSwerveModule::turningMotorConfig);

    m_turningNeoEncoder = m_turningMotor.builtinEncoder();
    m_driveNeoEncoder = m_driveMotor.builtinEncoder();
    Timer.delay(0.02);
    m_driveNeoEncoder.setPosition(0.0);
    m_turningEncoder = new SparkDutyCycleSensor(m_turningMotor);
    m_turningOffset = turningOffset;

    m_driveFeedforward = new SimpleMotorFeedforward(DrivetrainConstants.kDriveFeedforward);

    m_driveController = m_driveMotor.velocityController(DrivetrainConstants.kDriveMotorPIDGains);
    m_turningController =
        m_turningMotor.positionController(
            DrivetrainConstants.kTurningMotorPIDGains, m_turningEncoder.getSensor());

    m_turningController.enableContinuousInput(-Math.PI, Math.PI);

    // Simulation init
    m_turningMotorSim = new CANSparkSim1(m_turningMotor);
    m_driveMotorSim = new CANSparkSim1(m_driveMotor);
    m_turningSensorSim = new SparkDutyCycleSensorSim(m_turningEncoder.getSensor());
    m_swerveSim =
        new SwerveModuleSim(
            DCMotor.getNEO(1).withReduction(DrivetrainConstants.kDriveMotorReduction),
            DrivetrainConstants.kDriveFeedforward.kv,
            DrivetrainConstants.kDriveFeedforward.ka,
            DCMotor.getNeo550(1).withReduction(DrivetrainConstants.kTurningModuleGearRatio),
            DrivetrainConstants.kTurningFeedforward.kv
                * 500, // Not sure why * 500, but that does stabilize it
            DrivetrainConstants.kTurningFeedforward.ka * 500);

    // Set the position to the absolute values
    resetEncoders();

    // Reset the angle to current angle
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
  }

  /**
   * Configure function for drive motor. Configures the spark max conversion factors, inversion, and
   * current limits based on java, plus it sets idle mode, output range, and periodic frame rates.
   * This is fed directly into the Spark constructions using the withInitializer() method.
   *
   * @param spark the spark max being configured
   * @param isInit true if this is the first initialization on robot power on, false generally means
   *     a reset occured.
   * @return true if configuration succeded
   */
  private static Boolean driveMotorConfig(CANSparkFlex spark, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = spark.getEncoder();

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(DrivetrainConstants.kDriveEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(DrivetrainConstants.kDriveEncoderVelocityFactor));

    // Set inversion
    errors +=
        SparkMaxUtils.check(
            CANSparkBaseExtensions.setInverted(spark, DrivetrainConstants.kDriveMotorInvert));

    // // Configure velocity decoding, 16ms filter, 2 taps (For hall sensor)
    // errors += SparkMaxUtils.check(enc.setMeasurementPeriod(16));
    // errors += SparkMaxUtils.check(enc.setAverageDepth(2));

    // Configure velocity decoding, 100ms + 64 taps average is the default
    // TODO: Continue looking at values here. Equation for delay time with BFD is:
    // (period / 2) + (N - 1) / 2 * 0.5ms where N = average depth
    // With defaults: (100 / 2) + (64 - 1) / 2 * 0.5 ms = 65.75ms
    // With new values: (32 / 2) + (8 - 1) / 2 * 0.5 ms = 17.75ms
    errors += SparkMaxUtils.check(CANSparkBaseExtensions.setFlexEncoderAverageDepth(spark, 8));
    errors += SparkMaxUtils.check(CANSparkBaseExtensions.setFlexEncoderSampleDelta(spark, 32));

    errors += SparkMaxUtils.check(spark.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(spark.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(spark.setClosedLoopRampRate(0.025));
    errors +=
        SparkMaxUtils.check(
            spark.setSmartCurrentLimit(DrivetrainConstants.kDriveMotorCurrentLimit));
    errors +=
        SparkMaxUtils.check(
            spark.setSecondaryCurrentLimit(DrivetrainConstants.kDriveMotorCurrentChop));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus0, 100));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus1, 10));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus2, 10));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus3, 200));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus4, 30000));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus5, 50));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus6, 200));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus7, 30000));

    return errors == 0;
  }

  /**
   * Configure function for turning motor. Configures the spark max conversion factors, inversion,
   * and current limits based on java, plus it sets idle mode, output range, and periodic frame
   * rates. This is fed directly into the Spark constructions using the withInitializer() method.
   *
   * @param spark the spark max being configured
   * @param isInit true if this is the first initialization on robot power on, false generally means
   *     a reset occured.
   * @return true if configuration succeded
   */
  private static Boolean turningMotorConfig(CANSparkFlex spark, Boolean isInit) {
    int errors = 0;
    CANSparkBaseHandle m_handle = new CANSparkBaseHandle(spark);
    RelativeEncoder enc = spark.getEncoder();
    SparkAbsoluteEncoder dcEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(DrivetrainConstants.kTurningEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(DrivetrainConstants.kTurningEncoderVelocityFactor));

    // Convert rotations to radians
    // Enable center aligned mode to go [-pi, pi]
    errors +=
        SparkMaxUtils.check(
            REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(m_handle.handle, 152, true)));
    errors +=
        SparkMaxUtils.check(
            dcEncoder.setPositionConversionFactor(Math.PI + Units.degreesToRadians((0.35122 / 2))));
    errors += SparkMaxUtils.check(dcEncoder.setVelocityConversionFactor(Math.PI));

    // Phase of the duty cycle encoder is opposite of the motor
    errors += SparkMaxUtils.check(dcEncoder.setInverted(!DrivetrainConstants.kTurningMotorInvert));

    // Set inversion
    errors +=
        SparkMaxUtils.check(
            CANSparkBaseExtensions.setInverted(spark, DrivetrainConstants.kTurningMotorInvert));

    errors += SparkMaxUtils.check(spark.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(spark.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(spark));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus0, 100));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus1, 25));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus2, 50));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus3, 200));
    errors +=
        SparkMaxUtils.check(SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus5, 8));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(spark, PeriodicFrame.kStatus6, 50));

    return errors == 0;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    if (!RobotConstants.kReducedTelemetryMode) {
      builder.bindChild("Drive Controller", m_driveController);
      builder.bindChild("Turn Controller", m_turningController);
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

      builder.bindChild("TurningEncoder", m_turningNeoEncoder);
      builder.bindChild("DriveEncoder", m_driveNeoEncoder);
      builder.addDoubleProperty("DriveEncoderScaled", this::getDrivePosition, null);
      builder.bindChild("TurningAbsolute", m_turningEncoder);
      builder.bindChild("TurningEncoder", m_turningNeoEncoder);
      builder.addBooleanProperty(
          "Testmode Enable", () -> m_testEnable, (val) -> m_testEnable = val);
      builder.addDoubleProperty("Drive Current", () -> m_driveMotor.getOutputCurrent(), null);
      builder.addDoubleProperty("Turning Current", () -> m_turningMotor.getOutputCurrent(), null);
      builder.bindChild("Turning Motor", m_turningMotor);
      builder.bindChild("Drive Motor", m_driveMotor);
    }
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveNeoEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()).plus(new Rotation2d(m_turningOffset)));
  }

  @Override
  public SwerveModuleState getActualState() {
    return new SwerveModuleState(
        m_driveNeoEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Set the desired module state. This includes (robot centric) turing theta in radians, and speed
   * in meters per second.
   *
   * @param desiredState the desired state to set the module to.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    m_voltageDriveMode = false;

    // Everything from this point on is relative to the spark (i.e. after offset)
    desiredState.angle = desiredState.angle.minus(Rotation2d.fromRadians(m_turningOffset));

    // desired state must be withing [-pi, pi]
    if (Math.abs(desiredState.angle.getRadians()) > Math.PI) {
      org.tinylog.Logger.tag("Swerve Module " + m_turningMotor.getDeviceId())
          .error("Desired state: {} degrees", desiredState.angle.getRadians());

      desiredState.angle = new Rotation2d(MathUtil.angleModulus(desiredState.angle.getRadians()));
    }

    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // Important: Use spark max sensor directly, not wrapped sensor
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

    m_desiredState = desiredState;
  }

  /**
   * Run a specific module setpoint. This includes (robot centric) turing theta in radians, and
   * speed in meters per second. Use this to run directly to a state without any call to optimize.
   * Used when running 254 setpoint planner.
   *
   * @param state the state to set the module to.
   */
  @Override
  public void runSetpoint(SwerveModuleState state) {
    m_voltageDriveMode = false;

    // Everything from this point on is relative to the spark (i.e. after offset)
    state.angle = state.angle.minus(Rotation2d.fromRadians(m_turningOffset));

    // desired state must be withing [-pi, pi]
    if (Math.abs(state.angle.getRadians()) > Math.PI) {
      org.tinylog.Logger.tag("Swerve Module " + m_turningMotor.getDeviceId())
          .error("Desired state: {} degrees", state.angle.getRadians());

      state.angle = new Rotation2d(MathUtil.angleModulus(state.angle.getRadians()));
    }

    m_desiredState = state;
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void holdHeading(double speedMetersPerSecond) {
    m_desiredState.speedMetersPerSecond = speedMetersPerSecond;
    m_voltageDriveMode = false;
  }

  @Override
  public void setVoltageAngle(double voltage, Rotation2d angle) {
    m_desiredState.speedMetersPerSecond = 0.0;
    m_desiredState.angle = angle.minus(Rotation2d.fromRadians(m_turningOffset));
    m_voltageDriveMode = true;
    m_voltageDrive = voltage;
  }

  @Override
  public void periodic() {
    if (m_voltageDriveMode) {
      m_driveMotor.setVoltage(m_voltageDrive);
    } else {
      var desiredState = m_desiredState;
      // Scale speed by cosine of angle error. This scales down movement perpendicular to the
      // desired
      // direction of travel that can occur when modules change directions. This results in smoother
      // driving.
      // TODO: Try driving with and without this
      desiredState.speedMetersPerSecond *=
          desiredState.angle.minus(new Rotation2d(m_turningEncoder.getPosition())).getCos();

      // Calculate the turning motor output from the turning PID controller.
      m_driveController.setReference(
          m_desiredState.speedMetersPerSecond,
          m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    }

    // TODO: Set the motor angle here
    m_turningController.setReference(m_desiredState.angle.getRadians());
  }

  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (!m_testEnable) {
      m_turningMotor.setVoltage(0.0);
    } else {
      periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    double vbus = RobotController.getBatteryVoltage();
    m_turningMotorSim.enable();
    m_driveMotorSim.enable();

    m_turningMotorSim.iterate(m_swerveSim.getTurningMotorVelocityRPM(), vbus, 0.02);
    m_driveMotorSim.iterate(m_swerveSim.getDriveMotorVelocityRPM(), vbus, 0.02);

    m_swerveSim.iterate(
        m_driveMotorSim.getAppliedOutput() * vbus,
        m_turningMotorSim.getAppliedOutput() * vbus,
        0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_swerveSim.getCurrentDrawAmps()));

    m_turningMotorSim.setMotorCurrent(m_swerveSim.getTurningMotorCurrentDrawAmps());
    m_driveMotorSim.setMotorCurrent(m_swerveSim.getDriveMotorCurrentDrawAmps());
  }

  @Override
  public void resetEncoders() {
    // 'seed' the encoder to avoid nasty bug in getting encoder data
    m_driveNeoEncoder.setPosition(0.0001);
    m_turningNeoEncoder.setPosition(
        new Rotation2d(m_turningEncoder.getPosition())
            .plus(new Rotation2d(m_turningOffset))
            .getRadians());
  }

  private double getDrivePosition() {
    double pos = m_driveNeoEncoder.getPosition();
    if (m_driveMotor.getLastError() != REVLibError.kOk) {
      SmartDashboard.putBoolean("hasLastError", true);
      return 0.0;
    }
    SmartDashboard.putBoolean("hasLastError", false);
    return pos;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(),
        new Rotation2d(m_turningEncoder.getPosition()).plus(new Rotation2d(m_turningOffset)));
  }

  @Override
  public void close() {
    m_turningMotor.close();
    m_driveMotor.close();
  }

  protected void simSetTurningMotorPosition(Rotation2d theta) {
    m_turningSensorSim.setPosition(theta.getRadians());
  }

  @Override
  public void setDriveCurrent(int currentAmps) {
    m_driveMotor.fastParameterSet(
        () -> {
          CANSparkBaseExtensions.setSmartCurrentLimitStallOnly(m_driveMotor, currentAmps);
          CANSparkBaseExtensions.setSecondaryCurrentLimit(m_driveMotor, currentAmps + 10);
        });
  }
}
