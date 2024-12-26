package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.testmode.SelfTest;
import frc.lib.testmode.TestableSubsystem;
import frc.lib.util.Faults;
import frc.lib.vendor.indicator.Blinkin;
import frc.lib.vendor.indicator.Blinkin.BlinkinLedMode;
import frc.lib.vendor.motorcontroller.SparkController;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkFlex.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Robot;
import frc.robot.constants.HardwareMap;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import org.tinylog.Logger;

public class Intake extends TestableSubsystem implements TelemetryNode {
  private enum IntakeStatus {
    Intaking,
    IntakeToFeeder,
    IntakeBackoff,
    Feeding,
    Outaking,
    Stopped,
  };

  // Hardware objects
  private final SparkFlex m_leaderSparkFlex;
  private final SparkFlex m_feederMotor;
  private final Encoder m_encoder;
  private final SparkController m_controller;
  private final DigitalInput m_intakeBeamBreak =
      new DigitalInput(HardwareMap.Intake.kIntakeBeamBrakeDigitalPort);
  private final DigitalInput m_feederBeamBreak =
      new DigitalInput(HardwareMap.Intake.kFeederBeamBrakeDigitalPort);
  private final Blinkin m_blinkin = new Blinkin(HardwareMap.kBlinkinPwmChannel);

  // Settings
  private boolean kBackoffDisable = false;
  private double kIntakeToFeederTimeout = 4.0;
  private double kFeederBackoffTimeout = 4.0;
  private final boolean kHasGamePiecePolarity = false;
  private double kFeedTimeSeconds = 1.5;
  private double kIntakeVoltage = 15.0;
  private double kIntakeFeedVoltage = 15.0;
  private double kFeederFeedVoltage = 15.0;
  private double kFeederIndexVoltage = 4.0;
  private double kFeederBackoffVoltage = -1.0;
  private double kIntakeBackoffVoltage = -1.0;
  private double kOutakeVoltage = 10.0;
  private final double kRumbleTime = 1.0;

  // State
  private final Timer m_timer = new Timer();
  private final Timer m_rumbleTimer = new Timer();
  private BooleanConsumer m_rumberFcn = null;
  public IntakeStatus m_intakeStatus = IntakeStatus.Stopped;

  // Mechanism Visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(1, 1);
  private final MechanismRoot2d m_mechRoot = m_mech2d.getRoot("intake", 0.6, 0.1);
  private final Color8Bit kGreen = new Color8Bit(0, 255, 0);
  private final Color8Bit kRed = new Color8Bit(255, 0, 0);
  private final Color8Bit kGrey = new Color8Bit(128, 128, 128);
  private final MechanismLigament2d m_mechIntake =
      m_mechRoot.append(new MechanismLigament2d("intake", 0.5, 0.0));
  private final MechanismLigament2d m_mechFeeder =
      m_mechRoot.append(new MechanismLigament2d("feeder", 0.5, 90.0, 0.75, kGrey));

  // Simulation

  public Intake() {
    m_leaderSparkFlex = new SparkFlex(HardwareMap.Intake.kCanIdLeader, "Intake Leader");
    m_leaderSparkFlex.initialize(
        (isInit) -> {
          m_leaderSparkFlex.setFrameStrategy(FrameStrategy.kPosition);
          m_leaderSparkFlex.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
          m_leaderSparkFlex.setSecondaryCurrentLimit(IntakeConstants.kSecondaryCurrentLimit);
          return SparkMaxUtils.check(m_leaderSparkFlex.setIdleMode(IdleMode.kBrake)) == 0;
        });

    m_encoder = m_leaderSparkFlex.builtinEncoder();
    m_controller = m_leaderSparkFlex.positionController(IntakeConstants.kPositionGains);

    m_feederMotor = new SparkFlex(HardwareMap.Intake.kFeederCanId, "Feeder");
    m_feederMotor.initialize(
        (isInit) -> {
          m_feederMotor.setFrameStrategy(FrameStrategy.kNoFeedback);
          m_feederMotor.setSmartCurrentLimit(80);
          m_feederMotor.setSecondaryCurrentLimit(110);
          return SparkMaxUtils.check(m_feederMotor.setIdleMode(IdleMode.kBrake)) == 0;
        });

    SmartDashboard.putData("IntakeMechanism", m_mech2d);
    m_rumbleTimer.stop();
    m_rumbleTimer.reset();

    Faults.subsystem("Intake").markInitialized();

    m_blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
  }

  public boolean intakeHasGamePiece() {
    return m_intakeBeamBreak.get() == kHasGamePiecePolarity;
  }

  public boolean feederHasGamePiece() {
    return m_feederBeamBreak.get() == kHasGamePiecePolarity;
  }

  public void stop() {
    m_leaderSparkFlex.setVoltage(0.0);
    m_feederMotor.setVoltage(0.0);
    m_intakeStatus = IntakeStatus.Stopped;
  }

  /** Intake from the floor */
  public void intake() {
    Logger.tag("Intake").trace("Intaking");
    m_leaderSparkFlex.setVoltage(kIntakeVoltage);
    m_feederMotor.setVoltage(0.0);
    m_intakeStatus = IntakeStatus.Intaking;
  }

  public void intakeUnlessHasGamepiece() {
    // Only intake if there is no gamepiece and the intake is in stopped state or outtaking
    if (intakeHasGamePiece() || feederHasGamePiece()) {
      return;
    }

    if (m_intakeStatus == IntakeStatus.Stopped || m_intakeStatus == IntakeStatus.Outaking) {
      m_leaderSparkFlex.setVoltage(kIntakeVoltage);
      m_feederMotor.setVoltage(0.0);
      m_intakeStatus = IntakeStatus.Intaking;
    }
  }

  public void feedthrough() {
    Logger.tag("Intake").trace("Feedthrough!");

    // Same as feed(), except stop the timer so it runs forever
    feed();
    m_timer.stop();
  }

  private void updateBlinkin() {
    switch (m_intakeStatus) {
      case Feeding:
        m_blinkin.setMode(BlinkinLedMode.FIXED_STROBE_WHITE);
        break;
      case IntakeBackoff:
      case IntakeToFeeder:
        m_blinkin.setMode(BlinkinLedMode.TWO_STROBE);
        break;
      case Intaking:
        m_blinkin.setMode(BlinkinLedMode.ONE_STROBE);
        break;
      case Outaking:
        m_blinkin.setMode(BlinkinLedMode.FIXED_RAINBOW_PARTY);
        break;
      default:
        if (intakeHasGamePiece() || feederHasGamePiece()) {
          m_blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
        } else {
          m_blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
        }
        break;
    }
  }

  /** Feed the gamepiece to the shooter */
  public void feed() {
    Logger.tag("Intake").trace("Feeding");
    m_leaderSparkFlex.setVoltage(kIntakeFeedVoltage);
    m_feederMotor.setVoltage(kFeederFeedVoltage);
    m_intakeStatus = IntakeStatus.Feeding;
    m_timer.restart();
  }

  /** Reverse the rollers, will this ever be used? */
  public void outake() {
    m_leaderSparkFlex.setVoltage(-kOutakeVoltage);
    m_feederMotor.setVoltage(-kOutakeVoltage);
    m_intakeStatus = IntakeStatus.Outaking;
  }

  @Override
  public void periodic() {
    switch (m_intakeStatus) {
        /* Feeding gamepiece through to the shooter for time. */
      case Feeding:
        if (m_timer.hasElapsed(kFeedTimeSeconds)) {
          stop();
          m_intakeStatus = IntakeStatus.Stopped;
        }
        break;

        /* Intaking a note when none is present */
      case Intaking:
        if (intakeHasGamePiece()) {
          m_timer.restart();
          m_feederMotor.setVoltage(kFeederIndexVoltage);
          m_intakeStatus = IntakeStatus.IntakeToFeeder;
        } else {
          break;
        }
        // break; fall through so feeder is checked right away

        /* Note is in the intake, but not yet to the feeder */
      case IntakeToFeeder:
        m_rumbleTimer.restart();
        if (feederHasGamePiece()) {
          if (kBackoffDisable) {
            stop();
            m_intakeStatus = IntakeStatus.Stopped;
            break;
          }
          m_feederMotor.setVoltage(kFeederBackoffVoltage);
          m_leaderSparkFlex.setVoltage(kIntakeBackoffVoltage);
          m_timer.restart();
          m_intakeStatus = IntakeStatus.IntakeBackoff;
        } else if (m_timer.hasElapsed(kIntakeToFeederTimeout)) {
          stop();
          m_intakeStatus = IntakeStatus.Stopped;
        }
        break;

        /* Note is in feeder and needs to back off slightly */
      case IntakeBackoff:
        if (!feederHasGamePiece() || m_timer.hasElapsed(kFeederBackoffTimeout)) {
          stop();
          m_intakeStatus = IntakeStatus.Stopped;
        }
        break;

      default:
        break;
    }
    updateMechanism2d();
    updateRumble();
    updateBlinkin();
  }

  public void setRumbleFunction(BooleanConsumer rumbleFcn) {
    m_rumberFcn = rumbleFcn;
  }

  private void updateRumble() {
    if (RobotState.isAutonomous()) {
      return;
    }
    boolean doRumble = m_rumbleTimer.get() > 0.0001;
    if (m_rumberFcn != null) {
      m_rumberFcn.accept(doRumble);
    }

    if (m_rumbleTimer.hasElapsed(kRumbleTime)) {
      m_rumbleTimer.stop();
      m_rumbleTimer.reset();
    }
  }

  private void updateMechanism2d() {
    // Update intake visualization
    double appliedOutput = 0.0;
    double feederAppliedOutput = 0.0;
    if (Robot.isSimulation()) {
      // Sim outputs a voltage in voltage mode for some reason...
      appliedOutput = m_leaderSparkFlex.getAppliedOutput() / 12.0;
      feederAppliedOutput = m_feederMotor.getAppliedOutput() / 12.0;
    } else {
      appliedOutput = m_leaderSparkFlex.getAppliedOutput();
      feederAppliedOutput = m_feederMotor.getAppliedOutput();
    }
    m_mechIntake.setLength(Math.abs(appliedOutput) * 0.2 + 0.01);
    m_mechFeeder.setLength(Math.abs(feederAppliedOutput) * 0.2 + 0.01);

    if (appliedOutput == 0.0) {
      m_mechIntake.setColor(kGrey);
    } else {
      m_mechIntake.setColor(appliedOutput > 0 ? kGreen : kRed);
    }

    if (feederAppliedOutput == 0.0) {
      m_mechFeeder.setColor(kGrey);
    } else {
      m_mechFeeder.setColor(feederAppliedOutput > 0 ? kGreen : kRed);
    }
  }

  public Command IntakeCommand() {
    return new InstantCommand(this::intake, this).withName("Intake");
  }

  public Command IntakeContinuousCommand() {
    return Commands.run(this::intakeUnlessHasGamepiece, this)
        .withName("IntakeContinuous")
        .repeatedly();
  }

  public boolean isStoppedOrOutaking() {
    return m_intakeStatus == IntakeStatus.Stopped || m_intakeStatus == IntakeStatus.Outaking;
  }

  public Command OutakeCommand() {
    return new InstantCommand(this::outake, this).withName("Outake");
  }

  public Command FeedCommandNoRequirements() {
    return new InstantCommand(this::feed).withName("Intake Feed NoReq");
  }

  public Command FeedCommand() {
    return new InstantCommand(this::feed, this).withName("Intake Feed");
  }

  public Command FeedThroughCommand() {
    return new InstantCommand(this::feedthrough, this).withName("Intake Feedthrough");
  }

  public Command StopCommandNoRequirements() {
    return new InstantCommand(this::stop).withName("Intake Stop NoReq");
  }

  public Command StopCommand() {
    return new InstantCommand(this::stop, this).withName("Intake Stop");
  }

  @Override
  public void toSafeState() {
    stop();
  }

  @Override
  public SelfTest selfTest() {
    return new SelfTest(Commands.waitSeconds(1), () -> true);
  }

  public boolean isShooting() {
    return m_intakeStatus == IntakeStatus.Feeding && !intakeHasGamePiece() && feederHasGamePiece();
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
    builder.bindChild("Leader Flex", m_leaderSparkFlex);
    builder.bindChild("Leader Flex Encoder", m_encoder);
    builder.bindChild("Leader Flex Controller", m_controller);
    builder.bindChild("Feeder Flex", m_feederMotor);
    builder.addStringProperty("Intake Status", () -> m_intakeStatus.name(), null);
    builder.addBooleanProperty("Game Piece Status", this::intakeHasGamePiece, null);
    builder.addBooleanProperty("Feeder Piece Status", this::feederHasGamePiece, null);
    builder.addDoubleProperty("Rumble time", () -> m_rumbleTimer.get(), null);
    builder.addBooleanProperty("Is Shooting", this::isShooting, null);
    // builder.bindSendableChild("Intake Mechanism", m_mech2d);

    if (!RobotConstants.kCompetitionMode) {
      builder.addBooleanProperty(
          "Controls/Intake",
          () -> false,
          (val) -> {
            intake();
          });
      builder.addBooleanProperty(
          "Controls/Stop",
          () -> false,
          (val) -> {
            stop();
          });
      builder.addDoubleProperty(
          "Controls/Voltage",
          () -> 0.0001,
          (val) -> {
            m_leaderSparkFlex.setVoltage(val);
          });
      builder.addDoubleProperty(
          "Controls/FeederVoltage",
          () -> 0.0001,
          (val) -> {
            m_feederMotor.setVoltage(val);
          });

      builder.addDoubleProperty(
          "Settings/IntakeVoltage", () -> kIntakeVoltage, (val) -> kIntakeVoltage = val);
      builder.addDoubleProperty(
          "Settings/FeedVoltage", () -> kIntakeFeedVoltage, (val) -> kIntakeFeedVoltage = val);
      builder.addDoubleProperty(
          "Settings/OutakeVoltage", () -> kOutakeVoltage, (val) -> kOutakeVoltage = val);
      builder.addDoubleProperty(
          "Settings/FeedTimeSeconds", () -> kFeedTimeSeconds, (val) -> kFeedTimeSeconds = val);
    }
  }
}
