package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import frc.lib.controller.Controller;
import frc.lib.controller.PIDGains;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.util.Constraints;

public class SparkController implements Controller {
  private static final int VELOCITY_GAIN_SLOT = 0;
  private static final int POSITION_GAIN_SLOT = 1;
  protected static final int SMART_MOTION_GAIN_SLOT = 2;
  // Cache gains so 'get' commands don't go out on CAN
  // 4 gain slots in spark max
  private final SparkFlex m_spark;
  private PIDGains m_gainsCached = new PIDGains();
  private SparkPIDController m_sparkMaxController;
  private Constraints m_constraintsCached = new Constraints(0.0, 0.0);
  private int m_slot;
  private ControlType m_controlType;
  private double m_reference = 0.0;

  private double m_minimumInput = 0.0;
  private double m_maximumInput = 0.0;
  private boolean m_continuous = false;

  protected SparkController(
      SparkFlex spark,
      SparkPIDController pid,
      ControlType type,
      PIDGains gains,
      double minOutput,
      double maxOutput) {
    m_spark = spark;
    m_sparkMaxController = pid;
    m_controlType = type;
    if (type == ControlType.kPosition) {
      m_slot = POSITION_GAIN_SLOT;
    } else if (type == ControlType.kVelocity) {
      m_slot = VELOCITY_GAIN_SLOT;
    } else if (type == ControlType.kSmartMotion) {
      m_slot = SMART_MOTION_GAIN_SLOT;
    }
    m_gainsCached = gains;

    // Add this to mutate list so it is reset if controller resets
    spark.mutate(
        (cansparkmax, firstCall) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(m_sparkMaxController.setP(gains.P, m_slot));
          errors += SparkMaxUtils.check(m_sparkMaxController.setI(gains.I, m_slot));
          errors += SparkMaxUtils.check(m_sparkMaxController.setD(gains.D, m_slot));
          errors +=
              SparkMaxUtils.check(
                  m_sparkMaxController.setOutputRange(minOutput, maxOutput, m_slot));
          return errors == 0;
        });
  }

  public void setP(double p) {
    m_gainsCached.P = p;
    m_sparkMaxController.setP(p, m_slot);
  }

  public void setI(double i) {
    m_gainsCached.I = i;
    m_sparkMaxController.setI(i, m_slot);
  }

  public void setD(double d) {
    m_gainsCached.D = d;
    m_sparkMaxController.setD(d, m_slot);
  }

  /**
   * Initialize a sendable for tuning the velocity controller PID constants at run time
   *
   * @param builder Builder passed in during initSendable
   */
  private void controllerInitSendable(TelemetryBuilder builder, int slot) {
    builder.setSmartDashboardType("PIDController");
    builder.setActuator(true);
    builder.addDoubleProperty(
        "p",
        () -> m_gainsCached.P,
        (val) -> {
          m_gainsCached.P = val;
          m_sparkMaxController.setP(val, slot);
        });
    builder.addDoubleProperty(
        "i",
        () -> m_gainsCached.I,
        (val) -> {
          m_gainsCached.I = val;
          m_sparkMaxController.setI(val, slot);
        });
    builder.addDoubleProperty(
        "d",
        () -> m_gainsCached.D,
        (val) -> {
          m_gainsCached.D = val;
          m_sparkMaxController.setD(val, slot);
        });

    if (slot != SMART_MOTION_GAIN_SLOT) {
      return;
    }

    builder.addDoubleProperty(
        "max accel",
        () -> m_constraintsCached.maxAcceleration,
        (val) -> {
          m_constraintsCached.maxAcceleration = val;
          m_sparkMaxController.setSmartMotionMaxAccel(val, SMART_MOTION_GAIN_SLOT);
        });

    builder.addDoubleProperty(
        "max velocity",
        () -> m_constraintsCached.maxVelocity,
        (val) -> {
          m_constraintsCached.maxVelocity = val;
          m_sparkMaxController.setSmartMotionMaxVelocity(val, SMART_MOTION_GAIN_SLOT);
        });

    builder.addDoubleProperty("reference", () -> m_reference, null);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    controllerInitSendable(builder, m_slot);
  }

  @Override
  public void setReference(double reference, double feedforward) {
    m_reference = reference;
    m_sparkMaxController.setReference(reference, m_controlType, m_slot, feedforward);
  }

  public double getReference() {
    return m_reference;
  }

  @Override
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    // Implementation requires this currently
    assert minimumInput <= 0.0;
    assert maximumInput >= 0.0;

    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    this.m_sparkMaxController.setPositionPIDWrappingEnabled(true);
    this.m_sparkMaxController.setPositionPIDWrappingMaxInput(maximumInput);
    this.m_sparkMaxController.setPositionPIDWrappingMinInput(minimumInput);
  }

  @Override
  public void disableContinuousInput() {
    m_continuous = false;
    this.m_sparkMaxController.setPositionPIDWrappingEnabled(false);
  }

  @Override
  public boolean isContinuousInputEnabled() {
    return m_continuous;
  }
}
