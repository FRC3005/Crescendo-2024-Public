package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkSim1;
import com.revrobotics.SimDynamics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controller.ArmFeedforward;
import frc.lib.electromechanical.AbsoluteEncoder;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.testmode.TestableSubsystem;
import frc.lib.util.Faults;
import frc.lib.util.RobotName;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkFlex.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.lib.vendor.sensor.AMT212F;
import frc.lib.vendor.sensor.AMT212F.Baud;
import frc.lib.vendor.sensor.ThroughBoreEncoder;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.HardwareMap;
import frc.robot.constants.RobotConstants;
import java.util.function.DoubleSupplier;
import org.tinylog.Logger;

public class Arm extends TestableSubsystem implements TelemetryNode {

  // Hardware objects
  private final SparkFlex m_motor;
  private final Encoder m_neoEncoder;
  private final AbsoluteEncoder m_absoluteEncoder;
  private final AbsoluteEncoder m_secondaryEncoder;
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ArmConstants.kGains.P,
          ArmConstants.kGains.I,
          ArmConstants.kGains.D,
          ArmConstants.kConstraints);

  // Settings
  private ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kFeedForward);
  private final String kUseNeoSwitchName = "Arm Use NEO Encoder";
  private final double kFailoverHoldTime = 2.0;

  // State
  private double m_setpoint = 0.0;
  private boolean m_underStageMode = false;
  private boolean m_inEncoderFailover = false;
  private final Timer m_failoverTimer = new Timer();

  // Visualization

  // Simulation
  private final CANSparkSim1 m_motorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          ArmConstants.kArmGearbox,
          ArmConstants.kGearReduction,
          ArmConstants.kMOI,
          ArmConstants.kArmLengthM,
          ArmConstants.kMinRotation,
          ArmConstants.kMaxRotation,
          true,
          ArmConstants.kStartingPosition,
          VecBuilder.fill(
              2.0
                  * Math.PI
                  / (42 * ArmConstants.kGearReduction * 100)) // Add noise with a std-dev of 1 tick
          );

  public Arm() {
    if (RobotName.isCompetitionBot()) {
      m_absoluteEncoder =
          new AMT212F(Baud.k19200, ArmConstants.kHardStopAngle, (Math.PI * 2.0) / 4.0, false);
      m_secondaryEncoder =
          new ThroughBoreEncoder(
              HardwareMap.Arm.kThroughBoreDigitalPort,
              -ArmConstants.kThroughBoreOffset,
              (Math.PI * 2.0) / 4.0,
              false);
    } else {
      m_absoluteEncoder =
          new ThroughBoreEncoder(
              HardwareMap.Arm.kThroughBoreDigitalPort,
              -ArmConstants.kThroughBoreOffset,
              (Math.PI * 2.0) / 4.0,
              false);
      m_secondaryEncoder = null;
    }

    SmartDashboard.putBoolean(kUseNeoSwitchName, false);
    m_failoverTimer.restart();

    boolean didInit = m_absoluteEncoder.waitForInit(10.0);

    if (!didInit || !m_absoluteEncoder.isConnected()) {
      Faults.subsystem("Arm").fatal("Through bore encoder is not connected!");
    }

    m_motor = new SparkFlex(HardwareMap.Arm.kCanId, "Arm");
    m_neoEncoder = m_motor.builtinEncoder();
    m_motor.initialize(this::sparkMaxInitializer);

    m_motorSim = new CANSparkSim1(m_motor, SimDynamics.fromSim(m_armSim));

    m_controller.setTolerance(Units.degreesToRadians(0.1));

    Faults.subsystem("Arm").markInitialized();
  }

  private Boolean sparkMaxInitializer(Boolean isInit) {
    var encoder = m_motor.getEncoder();
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(m_motor));
    errors += SparkMaxUtils.check(m_motor.setIdleMode(IdleMode.kBrake));

    if (!m_absoluteEncoder.isConnected()) {
      Logger.tag("Arm")
          .warn(
              "No through bore! Setting spark encoder to position: {}",
              ArmConstants.kStartingPosition);
      if (isInit) {
        errors += SparkMaxUtils.check(encoder.setPosition(ArmConstants.kStartingPosition));
        errors += SparkMaxUtils.check(encoder.setPosition(ArmConstants.kStartingPosition));
        errors += SparkMaxUtils.check(encoder.setPosition(ArmConstants.kStartingPosition));
        m_inEncoderFailover = true;
      }
    } else {
      var pos = getPositionRadians();
      Logger.tag("Arm").info("Setting spark encoder to position: {}", pos);
      errors += SparkMaxUtils.check(m_motor.getEncoder().setPosition(pos));
      errors += SparkMaxUtils.check(m_motor.getEncoder().setPosition(pos));
    }

    errors +=
        SparkMaxUtils.check(
            m_motor.setSoftLimit(
                SoftLimitDirection.kForward, (float) ArmConstants.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            m_motor.setSoftLimit(
                SoftLimitDirection.kReverse, (float) ArmConstants.kReverseSoftLimit));

    errors +=
        SparkMaxUtils.check(
            encoder.setPositionConversionFactor(2 * Math.PI / ArmConstants.kGearReduction));

    errors +=
        SparkMaxUtils.check(CANSparkBaseExtensions.setInverted(m_motor, ArmConstants.kMotorInvert));
    m_motor.setFrameStrategy(FrameStrategy.kVelocityAndPosition);
    resetEncoder();
    return errors == 0;
  }

  // Position in radians
  public void setPosition(double position) {
    m_setpoint = position;
  }

  public void setPosition(Rotation2d position) {
    setPosition(position.getRadians());
  }

  /**
   * @return true when the profile completed
   */
  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  /**
   * @param tolerance delta to position to call at setpoint
   * @return true if within tolerance _and_ profile completed
   */
  public boolean atSetpoint(double tolerance) {
    if (Math.abs(getPositionRadians() - m_setpoint) < tolerance) {
      return true;
    }
    return false;
  }

  public Command setPositionCommand(Rotation2d position) {
    return Commands.run(() -> setPosition(position), this)
        .until(this::atSetpoint)
        .withName("Set Position");
  }

  public Command setLowestPositionCommand() {
    return setPositionCommand(Rotation2d.fromRadians(ArmConstants.kHardStopAngle));
  }

  public Command trackGoalCommand(DoubleSupplier supplier) {
    return Commands.run(() -> setPosition(supplier.getAsDouble()), this).withName("Tracking");
  }

  private boolean m_previouslyDisabled = true;
  private double m_profileVoltage = 0.0;

  @Override
  public void periodic() {
    if (m_setpoint > ArmConstants.kForwardSoftLimit) {
      m_setpoint = ArmConstants.kForwardSoftLimit;
    } else if (m_setpoint < ArmConstants.kReverseSoftLimit) {
      m_setpoint = ArmConstants.kReverseSoftLimit;
    }

    // Limit this to autonomous
    if (m_underStageMode && RobotState.isAutonomous()) {
      if (m_setpoint > ArmConstants.kUnderStageLimit) {
        m_setpoint = ArmConstants.kUnderStageLimit;
      }
    }

    double position = shouldFailover() ? getNeoPositionRadians() : getPositionRadians();

    if (RobotState.isDisabled()) {
      m_previouslyDisabled = true;
    } else if (m_previouslyDisabled) {
      m_previouslyDisabled = false;
      m_controller.reset(position);
    }

    m_profileVoltage = m_controller.calculate(position, m_setpoint);
    var state = m_controller.getSetpoint();
    m_motor.setVoltage(m_profileVoltage + m_feedforward.calculate(state.position, state.velocity));
  }

  private double getPositionRadians() {
    double pos = m_absoluteEncoder.getPosition();
    if (pos < Units.degreesToRadians(10.0)) {
      return Units.degreesToRadians(90.0) + pos;
    }
    return pos;
  }

  private double getNeoPositionRadians() {
    return m_neoEncoder.getPosition();
  }

  private boolean shouldFailover() {
    boolean dashboardFailover = SmartDashboard.getBoolean(kUseNeoSwitchName, false);
    boolean failoverTimeElapsed = m_failoverTimer.hasElapsed(kFailoverHoldTime);

    if (dashboardFailover) {
      m_inEncoderFailover = true;
      m_failoverTimer.restart();
    } else if (!m_absoluteEncoder.isConnected() && failoverTimeElapsed && !m_inEncoderFailover) {
      m_inEncoderFailover = true;
      m_failoverTimer.restart();
      Logger.tag("Arm").error("Failing over to NEO encoder!");
    } else if (m_absoluteEncoder.isConnected() && failoverTimeElapsed && m_inEncoderFailover) {
      m_inEncoderFailover = false;
      m_failoverTimer.restart();
      Logger.tag("Arm").warn("Switching back to absolute encoder");
    }

    return m_inEncoderFailover;
  }

  public void setUnderStageMode(boolean enable) {
    m_underStageMode = enable;
  }

  public Command enableArmUnderStageCommand() {
    return Commands.runOnce(() -> setUnderStageMode(true));
  }

  public Command disableArmUnderStageCommand() {
    return Commands.runOnce(() -> setUnderStageMode(false));
  }

  public void resetEncoder() {
    if (!m_absoluteEncoder.isConnected()) {
      Logger.tag("Arm").error("Through bore sensor not connected, skipping reset!");
      return;
    }

    m_setpoint = getPositionRadians();

    /*
     * Disable the arm motor briefly such that the arm doesn't
     * jump while waiting for the setpoint to be set.
     */
    m_motor.set(0);
    m_neoEncoder.setPosition(m_setpoint);
    m_neoEncoder.setPosition(m_setpoint);
    Timer.delay(0.005);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    double voltage = RobotController.getBatteryVoltage();
    double angle = m_armSim.getAngleRads();
    m_armSim.setInput(m_controller.calculate(angle) * voltage);

    // Next, we update it. The standard loop time is 20ms.
    // m_motorSim.update(0.020);
    m_armSim.update(0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    // m_simMech.setArm(m_armSim.getAngleRads());

    // REV Sim is totally broken, just force its position and see if that works :(
    m_motorSim.setPosition(angle);
  }

  // https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md
  private final double[] m_poseViz = new double[7];

  private double[] getArmViz() {
    Rotation3d r = new Rotation3d(0.0, (getPositionRadians() - Units.degreesToRadians(18.0)), 0.0);
    Quaternion q = r.getQuaternion();
    m_poseViz[0] = -0.22225;
    m_poseViz[2] = 0.3302;
    m_poseViz[3] = q.getW();
    m_poseViz[4] = q.getX();
    m_poseViz[5] = q.getY();
    m_poseViz[6] = q.getZ();
    return m_poseViz;
  }

  private SysIdRoutine sysIdRoutineDynamic() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Seconds.of(1.0)), Volts.of(4.0), Seconds.of(1.0)),
        new SysIdRoutine.Mechanism((voltage) -> m_motor.setVoltage(voltage.in(Volts)), null, this));
  }

  private SysIdRoutine sysIdRoutineQuasistatic() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(1.0)), Volts.of(1.0), Seconds.of(2.0)),
        new SysIdRoutine.Mechanism((voltage) -> m_motor.setVoltage(voltage.in(Volts)), null, this));
  }

  public Command sysId() {
    return Commands.sequence(
        setPositionCommand(Rotation2d.fromDegrees(20.0)),
        Commands.waitSeconds(2.0),
        sysIdRoutineQuasistatic().quasistatic(Direction.kForward),
        Commands.waitSeconds(2.0),
        setPositionCommand(Rotation2d.fromDegrees(90.0)),
        Commands.waitSeconds(2.0),
        sysIdRoutineQuasistatic().quasistatic(Direction.kReverse),
        Commands.waitSeconds(2.0),
        setPositionCommand(Rotation2d.fromDegrees(20.0)),
        Commands.waitSeconds(2.0),
        sysIdRoutineDynamic().dynamic(Direction.kForward),
        Commands.waitSeconds(2.0),
        setPositionCommand(Rotation2d.fromDegrees(90.0)),
        Commands.waitSeconds(2.0),
        sysIdRoutineDynamic().dynamic(Direction.kReverse),
        setPositionCommand(Rotation2d.fromDegrees(20.0)));
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    this.initSendable(builder);
    builder.bindChild("NEO Encoder", m_neoEncoder);
    builder.bindChild("Abs Encoder", m_absoluteEncoder);
    builder.bindChild("Motor", m_motor);
    builder.addBooleanProperty("In Encoder Failover", () -> m_inEncoderFailover, null);
    builder.addDoubleProperty(
        "Abs Sensor Angle Deg", () -> Units.radiansToDegrees(getPositionRadians()), null);
    if (m_secondaryEncoder != null) {
      builder.addDoubleProperty(
          "Through Bore Angle Deg",
          () -> Units.radiansToDegrees(m_secondaryEncoder.getPosition()),
          null);
    }
    builder.addDoubleProperty(
        "Neo Encoder Angle Deg", () -> Units.radiansToDegrees(m_neoEncoder.getPosition()), null);
    builder.bindSendableChild("Controller", m_controller);
    builder.addDoubleProperty("Controller Voltage", () -> m_profileVoltage, null);

    if (!RobotConstants.kReducedTelemetryMode) {
      builder.addDoubleProperty(
          "Setpoint Deg",
          () -> Units.radiansToDegrees(m_setpoint),
          (val) -> m_setpoint = Units.degreesToRadians(val));
      builder.addDoubleArrayProperty("Launcher Pose", this::getArmViz, null);

      // Controls
      builder.addBooleanProperty("Reset Encoder", () -> false, (val) -> resetEncoder());

      // Settings
      builder.addDoubleProperty("ks", () -> m_feedforward.ks, (ks) -> m_feedforward.ks = ks);
      builder.addDoubleProperty("kg", () -> m_feedforward.kg, (kg) -> m_feedforward.kg = kg);
      builder.addDoubleProperty("kv", () -> m_feedforward.kv, (kv) -> m_feedforward.kv = kv);
      builder.addDoubleProperty("ka", () -> m_feedforward.ka, (ka) -> m_feedforward.ka = ka);

      builder.addBooleanProperty(
          "Under Stage Mode", () -> m_underStageMode, (val) -> m_underStageMode = val);
    }

    if (Robot.isSimulation()) {
      builder.addDoubleProperty("Sim Arm Angle", () -> m_armSim.getAngleRads(), null);
    }
  }
}
