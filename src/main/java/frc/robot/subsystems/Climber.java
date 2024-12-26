package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkSim1;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.testmode.TestableSubsystem;
import frc.lib.util.Faults;
import frc.lib.vendor.motorcontroller.SparkController;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkFlex.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.HardwareMap;

public class Climber extends TestableSubsystem implements TelemetryNode {

  private enum ClimberState {
    Stopped,
    Extending,
    Retracting,
  };

  // Hardware
  private final SparkFlex m_climberMotor = new SparkFlex(HardwareMap.Climber.kCanId, "Climber");
  private final Encoder m_neoEncoder = m_climberMotor.builtinEncoder();
  private final SparkController m_controller =
      m_climberMotor.positionController(ClimberConstants.kPIDGains);
  private final Servo m_lockServo = new Servo(HardwareMap.Climber.kServoPort);

  // State
  private final Timer m_servoTimer = new Timer();
  private ClimberState m_state = ClimberState.Stopped;
  private ClimberState m_desiredState = ClimberState.Stopped;

  // Sim
  private final CANSparkSim1 m_sim = new CANSparkSim1(m_climberMotor);

  public Climber() {
    m_servoTimer.restart();
    m_lockServo.set(0.0);
    m_climberMotor.initialize(this::sparkMaxInitializer);
    Faults.subsystem("Climber").markInitialized();
  }

  private Boolean sparkMaxInitializer(Boolean isInit) {
    var encoder = m_climberMotor.getEncoder();
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(m_climberMotor));
    errors += SparkMaxUtils.check(m_climberMotor.setIdleMode(IdleMode.kBrake));
    errors +=
        SparkMaxUtils.check(
            CANSparkBaseExtensions.setInverted(m_climberMotor, ClimberConstants.kInverted));

    errors +=
        SparkMaxUtils.check(
            m_climberMotor.setSoftLimit(
                SoftLimitDirection.kForward, (float) ClimberConstants.kTopPosition));
    errors +=
        SparkMaxUtils.check(
            m_climberMotor.setSoftLimit(
                SoftLimitDirection.kReverse, (float) ClimberConstants.kStartingPosition));

    errors +=
        SparkMaxUtils.check(
            encoder.setPositionConversionFactor(
                (Math.PI * ClimberConstants.kDrumDiameter) / ClimberConstants.kGearReduction));

    errors += SparkMaxUtils.check(encoder.setPosition(0.0));

    m_climberMotor.setFrameStrategy(FrameStrategy.kNoFeedback);
    SparkMaxUtils.setPeriodicFramePeriod(m_climberMotor, PeriodicFrame.kStatus2, 100);
    return errors == 0;
  }

  public boolean isServoOpen() {
    // Just assume its locked right away
    return m_lockServo.get() > 0.5 && m_servoTimer.hasElapsed(ClimberConstants.kUnlockTimeSeconds);
  }

  public boolean isServoLocked() {
    return !isServoOpen();
  }

  private void retract() {
    if (isServoLocked()) {
      unlockServo();
    }
    m_desiredState = ClimberState.Retracting;
  }

  private void extend() {
    if (isServoLocked()) {
      unlockServo();
    }
    m_desiredState = ClimberState.Extending;
  }

  public void stop() {
    m_desiredState = ClimberState.Stopped;
    m_climberMotor.set(0.0);
  }

  private void unlockServo() {
    if (m_lockServo.get() < 0.5) {
      // Restart timer only when going from lock to unlock
      m_servoTimer.restart();
    }
    m_lockServo.set(1.0);
  }

  private void lockServo() {
    if (m_lockServo.get() > 0.5) {
      // Restart timer only when going from unlock to lock
      m_servoTimer.restart();
    }
    m_lockServo.set(0.0);
  }

  private boolean atSetpoint() {
    return Math.abs(m_controller.getReference() - m_neoEncoder.getPosition())
            < Units.inchesToMeters(1.5)
        && m_neoEncoder.getVelocity() < 10.0;
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Stop Command");
  }

  public Command extendCommand() {
    return Commands.runOnce(this::extend, this).withName("Extend Command");
  }

  public Command retractCommand() {
    return Commands.runOnce(this::retract, this).withName("Retract Command");
  }

  public Command lockServoCommand() {
    return Commands.sequence(
            stopCommand(),
            Commands.waitSeconds(ClimberConstants.kLockAfterClimbTimeSeconds),
            Commands.runOnce(this::lockServo, this),
            Commands.waitUntil(this::isServoLocked))
        .withName("Lock Servo Command");
  }

  @Override
  public void periodic() {
    if (isServoLocked()) {
      m_state = ClimberState.Stopped;
    } else {
      m_state = m_desiredState;
    }

    switch (m_state) {
      case Extending:
        m_controller.setReference(ClimberConstants.kTopPosition);
        if (atSetpoint()) {
          stop();
        }
        break;

      case Retracting:
        m_climberMotor.set(ClimberConstants.kRetractSpeed);
        if (m_neoEncoder.getPosition() <= ClimberConstants.kStartingPosition) {
          stop();
        }
        break;

      case Stopped:
      default:
        m_climberMotor.set(0.0);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_sim.iterate(12.0, 0.02);
  }

  @Override
  public void toSafeState() {
    m_climberMotor.set(0.0);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    this.initSendable(builder);
    builder.bindChild("NEO Encoder", m_neoEncoder);
    builder.bindChild("Motor", m_climberMotor);
    builder.bindChild("Coontroller", m_controller);
    builder.bindSendableChild("Lock Servo", m_lockServo);
    builder.addStringProperty("State", m_state::name, null);
    builder.addStringProperty("DesiredState", m_desiredState::name, null);
    builder.addBooleanProperty("Servo Locked", () -> isServoLocked(), null);
  }
}
