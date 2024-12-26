package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.testmode.SelfTest;
import frc.lib.testmode.TestableSubsystem;
import frc.lib.util.Faults;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.RobotConstants;
import java.util.function.Supplier;

public class Launcher extends TestableSubsystem implements TelemetryNode {

  // Hardware objects
  // Front Left (FL), Front Right(FR), BackLeft (BL), BackRight (BR)
  private final LauncherWheel m_topLeftWheel = new LauncherWheel(LauncherWheel.Location.TopLeft);
  private final LauncherWheel m_topRightWheel = new LauncherWheel(LauncherWheel.Location.TopRight);
  private final LauncherWheel m_bottomLeftWheel =
      new LauncherWheel(LauncherWheel.Location.BottomLeft);
  private final LauncherWheel m_bottomRightWheel =
      new LauncherWheel(LauncherWheel.Location.BottomRight);
  private final LauncherWheel[] m_allLaunchers =
      new LauncherWheel[] {
        m_topLeftWheel, m_topRightWheel, m_bottomLeftWheel, m_bottomRightWheel,
      };

  // Settings
  // Order must be TL, TR, BL, BR
  private double[] kLauncherRpmsSpeakerClose = new double[] {2109.0, 3143.0, 2109.0, 3143.0};
  // private double[] kLauncherRpms = new double[] {500.0, 1000.0, 500.0, 1000.0};
  private double kAmpRpm = 1428.0;
  private double kIdleRpm = LauncherConstants.kIdleRpm;

  // State

  // Mechanism Visualization

  // Simulation

  public Launcher() {
    Faults.subsystem("Launcher").markInitialized();
  }

  public void stop() {
    for (var launcher : m_allLaunchers) {
      launcher.setVoltage(0.0);
    }
  }

  public void idle() {
    for (var launcher : m_allLaunchers) {
      launcher.setVelocity(kIdleRpm);
    }
  }

  /** Runs motors to launch gamepiece */
  public void shootClose() {
    for (int i = 0; i < m_allLaunchers.length; i++) {
      m_allLaunchers[i].setVelocity(kLauncherRpmsSpeakerClose[i]);
    }
  }

  public void shoot(double velocityLeft, double velocityRight) {
    m_bottomLeftWheel.setVelocity(velocityLeft);
    m_topLeftWheel.setVelocity(velocityLeft);
    m_bottomRightWheel.setVelocity(velocityRight);
    m_topRightWheel.setVelocity(velocityRight);
  }

  public void ampShot() {
    for (var launcher : m_allLaunchers) {
      launcher.setVelocity(kAmpRpm);
    }
  }

  public Command shootCloseCommand() {
    return new InstantCommand(this::shootClose, this).withName("Shoot CLose");
  }

  public Command shootTuningShotCommand() {
    return new InstantCommand(
        () -> {
          LauncherSetpoint setpoint =
              new LauncherSetpoint(m_tuningShotRpm / m_tuningRatio, m_tuningShotRpm);
          shoot(setpoint.leftRpm, setpoint.rightRpm);
        },
        this);
  }

  public Command shootFixedRpm(LauncherSetpoint setpoint) {
    return new InstantCommand(() -> shoot(setpoint.leftRpm, setpoint.rightRpm), this)
        .withName("Shoot Fixed RPM");
  }

  public Command shootTrackCommand(Supplier<LauncherSetpoint> supplier) {
    return new RunCommand(
            () -> {
              LauncherSetpoint setpoint = supplier.get();
              shoot(setpoint.leftRpm, setpoint.rightRpm);
            },
            this)
        .withName("Shoot Track");
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this).withName("Shoot Stop");
  }

  public Command idleCommand() {
    return new InstantCommand(this::idle, this).withName("Shoot Idle");
  }

  public Command ampCommand() {
    return new InstantCommand(this::ampShot, this).withName("Shoot Amp");
  }

  public boolean atSetpoint(double toleranceRpm) {
    for (var launcher : m_allLaunchers) {
      if (!launcher.atSetpoint(toleranceRpm)) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void toSafeState() {
    stop();
  }

  @Override
  public SelfTest selfTest() {
    return new SelfTest(Commands.waitSeconds(1), () -> true);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    double voltage = RobotController.getBatteryVoltage();
    double current = 0.0;
    for (var launcher : m_allLaunchers) {
      current += launcher.calculateSim(voltage);
    }

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(current));
  }

  private SysIdRoutine sysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1.0).per(Seconds.of(1.0)), Volts.of(6.0), Seconds.of(10.0)),
        new SysIdRoutine.Mechanism(
            (voltage) -> m_topLeftWheel.setVoltage(voltage.in(Volts)), null, this));
  }

  public Command sysId() {
    return Commands.sequence(
        sysIdRoutine().quasistatic(Direction.kForward),
        Commands.waitSeconds(3.0),
        sysIdRoutine().quasistatic(Direction.kReverse),
        Commands.waitSeconds(3.0),
        sysIdRoutine().dynamic(Direction.kForward),
        Commands.waitSeconds(3.0),
        sysIdRoutine().dynamic(Direction.kReverse));
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);

    for (var launcher : m_allLaunchers) {
      launcher.bind(builder);
    }

    if (!RobotConstants.kCompetitionMode) {
      builder.addDoubleProperty(
          "Voltage All",
          () -> 0.001,
          (val) -> {
            for (var launcher : m_allLaunchers) {
              launcher.setVoltage(val);
            }
          });
      builder.addDoubleProperty("Amp RPMs", () -> kAmpRpm, (val) -> kAmpRpm = val);
      builder.addDoubleProperty("Idle RPMs", () -> kAmpRpm, (val) -> kAmpRpm = val);
      builder.addDoubleProperty(
          "Tuning Shot Ratio", () -> m_tuningRatio, (val) -> m_tuningRatio = val);
      builder.addDoubleProperty(
          "Tuning Shot RPM", () -> m_tuningShotRpm, (val) -> m_tuningShotRpm = val);
    }
  }

  private double m_tuningRatio = LauncherConstants.kShooterRatio;
  private double m_tuningShotRpm = 2000;
}
