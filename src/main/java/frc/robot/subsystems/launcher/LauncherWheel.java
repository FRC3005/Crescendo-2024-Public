package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkSim1;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controller.Controller;
import frc.lib.controller.PIDControllerWrapper;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Robot;
import frc.robot.constants.HardwareMap;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.RobotConstants;

public class LauncherWheel {
  // Hardware
  private final SparkFlex m_motor;
  private final Encoder m_encoder;
  private final Controller m_controller;

  // Settings
  private final Location m_location;
  private final String m_name;

  // State
  private double m_setpoint = 0.0;

  // Visualization

  // Sim
  private final CANSparkSim1 m_sparkSim;
  private final PIDController m_simController =
      new PIDController(
          LauncherConstants.kPidGains.P,
          LauncherConstants.kPidGains.I,
          LauncherConstants.kPidGains.D);
  private final PIDControllerWrapper m_simControllerWrapper =
      new PIDControllerWrapper(m_simController);
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV);
  private final FlywheelSim m_sim =
      new FlywheelSim(
          DCMotor.getNeoVortex(1), LauncherConstants.kGearRatio, LauncherConstants.kMOI);

  public enum Location {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight;

    private final int kCanIdMap[] =
        new int[] {
          HardwareMap.Launcher.kCanIdTL,
          HardwareMap.Launcher.kCanIdTR,
          HardwareMap.Launcher.kCanIdBL,
          HardwareMap.Launcher.kCanIdBR,
        };

    // Must be same order as enum
    private final boolean kInvertMap[] =
        new boolean[] {
          false, // TL
          true, // TR
          true, // BL
          false // BR
        };

    public int canId() {
      return kCanIdMap[this.ordinal()];
    }

    public boolean invert() {
      return kInvertMap[this.ordinal()];
    }
  };

  public LauncherWheel(Location location) {
    m_motor =
        new SparkFlex(location.canId(), location.name())
            .withInitializer(
                (spark, isInit) -> {
                  int errors = 0;
                  errors += SparkMaxUtils.check(spark.setIdleMode(IdleMode.kCoast));

                  // Configure velocity decoding, 100ms + 64 taps average is the default
                  // TODO: Continue looking at values here. Equation for delay time with BFD is:
                  // (period / 2) + (N - 1) / 2 * 0.5ms where N = average depth
                  // With defaults: (100 / 2) + (64 - 1) / 2 * 0.5 ms = 65.75ms
                  // With new values: (32 / 2) + (8 - 1) / 2 * 0.5 ms = 17.75ms
                  errors +=
                      SparkMaxUtils.check(
                          CANSparkBaseExtensions.setFlexEncoderAverageDepth(spark, 8));
                  errors += SparkMaxUtils.check(spark.setSmartCurrentLimit(45));
                  errors += SparkMaxUtils.check(spark.setSecondaryCurrentLimit(75));
                  errors +=
                      SparkMaxUtils.check(
                          CANSparkBaseExtensions.setFlexEncoderSampleDelta(spark, 32));

                  errors +=
                      SparkMaxUtils.check(CANSparkBaseExtensions.disablePIDVoltageOutput(spark));

                  errors +=
                      SparkMaxUtils.check(
                          CANSparkBaseExtensions.setInverted(spark, location.invert()));
                  return errors == 0;
                });
    m_encoder = m_motor.builtinEncoder();

    if (Robot.isSimulation()) {
      m_controller = m_simControllerWrapper;
    } else {
      m_controller = m_motor.velocityController(LauncherConstants.kPidGains, 0.0, 1.0);
    }

    m_location = location;
    m_name = "Launcher" + location.name();
    m_sparkSim = new CANSparkSim1(m_motor);
  }

  public void setVelocity(double rpm) {
    m_setpoint = rpm;
    m_controller.setReference(rpm, m_feedforward.calculate(rpm));
  }

  public double getVelocity() {
    var velocity = m_encoder.getVelocity();
    if (m_motor.getLastError() != REVLibError.kOk) {
      SmartDashboard.putBoolean("LauncherSkip", true);
      return LauncherConstants.kIdleRpm;
    }
    return velocity;
  }

  public String getName() {
    return m_name;
  }

  // Only use this for testing
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  public boolean atSetpoint(double toleranceRpm) {
    // Symmetric
    // return Math.abs(getVelocity() - m_setpoint) <= toleranceRpm;

    // Asymmetric is probably fine
    return (getVelocity() >= (m_setpoint - toleranceRpm))
        && m_setpoint != LauncherConstants.kIdleRpm;
  }

  public double calculateSim(double voltage) {
    double velocity = m_sim.getAngularVelocityRPM();
    double output = m_simController.calculate(velocity);
    if (output < 0) {
      output = 0;
    }
    m_sim.setInputVoltage(output * voltage);
    m_sim.update(0.02);
    m_sparkSim.setVelocity(velocity);
    return m_sim.getCurrentDrawAmps();
  }

  public void bind(TelemetryBuilder builder) {
    String prefix = "Wheel/" + m_location.name() + "/";
    builder.bindChild(prefix + "Encoder", m_encoder);
    builder.bindChild(prefix + "Controller", m_controller);
    if (!RobotConstants.kReducedTelemetryMode) {
      builder.addDoubleProperty(prefix + "Setpoint", () -> m_setpoint, this::setVelocity);
    }
  }
}
