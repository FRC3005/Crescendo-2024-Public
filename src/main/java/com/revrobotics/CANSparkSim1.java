package com.revrobotics;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.lib.util.MovingAverageFilterSim;
import frc.lib.util.NoiseGenerator;

public class CANSparkSim1 {
  private final SimDouble m_appliedOutput;
  private final SimDouble m_velocity;
  private final SimDouble m_position;
  private final SimDouble m_busVoltage;
  private final SimDouble m_motorCurrent;
  private final SimDouble m_setpoint;
  private final SimDouble m_arbFF;
  private final SimInt m_faults;
  private final SimInt m_stickyFaults;
  private final SimInt m_pidSlot;
  private final SimInt m_arbFFUnits;
  private final CANSparkBase m_spark;
  private final SimDynamics m_simulatedDynamics;
  private final SimInt m_controlMode;
  private boolean m_forwardLimit = false;
  private boolean m_reverseLimit = false;
  private final MovingAverageFilterSim m_velocityAverage = new MovingAverageFilterSim(2, 0.016);
  // private final MovingAverageFilterSim m_velocityAverage = new MovingAverageFilterSim(8, 0.032);
  private Boolean m_enable = null;

  // PID State
  double m_iState = 0.0;
  double m_prev_err = 0.0;
  double m_pidOutput = 0.0;

  /**
   * This *should* track to the API version. Since this file is dependent on REV, warn if the
   * version changes to go back and verify 1) this still works and 2) that this file is still needed
   * at all.
   */
  private final int kAPIversionExpected = 132627716;

  /**
   * Create a simulated CAN Spark object. This class simulates some of the internal behavior of the
   * device.
   *
   * @param spark The CANSparkBase to simulate.
   * @param FlywheelSim A basic dynamics model. If you are not modeling the system externally this
   *     can be used instead. If you have a second motor controller as a follower, you can use a
   *     single CANSparkSim object on the leader, and specify multiple motors using the DCMotor
   *     constructor.
   */
  public CANSparkSim1(CANSparkBase spark, SimDynamics dynamicsSim) {
    SimDeviceSim sparkSim = new SimDeviceSim("SPARK MAX" + " [" + spark.getDeviceId() + "]");
    m_appliedOutput = sparkSim.getDouble("Applied Output");
    m_position = sparkSim.getDouble("Position");
    m_velocity = sparkSim.getDouble("Velocity");
    m_busVoltage = sparkSim.getDouble("Bus Voltage");
    m_motorCurrent = sparkSim.getDouble("Motor Current");
    m_setpoint = sparkSim.getDouble("Setpoint");
    m_arbFF = sparkSim.getDouble("Arbitrary Feedforward");
    m_pidSlot = sparkSim.getInt("PID Slot");
    m_arbFFUnits = sparkSim.getInt("ArbFF Units");
    m_controlMode = sparkSim.getInt("Control Mode");
    m_faults = sparkSim.getInt("Faults");
    m_stickyFaults = sparkSim.getInt("Sticky Faults");
    m_spark = spark;
    m_simulatedDynamics = dynamicsSim;

    // int apiVersion = CANSparkMaxJNI.c_SparkMax_GetAPIVersion();
    // if (apiVersion != kAPIversionExpected) {
    //   Logger.tag("CANSparkSim")
    //       .trace(
    //           "CAN Spark API version changed, verify that the sim setup still works correctly.
    // Got {} expected {}",
    //           apiVersion,
    //           kAPIversionExpected);
    // }
  }

  /**
   * Create a simulated CAN Spark object. This class simulates some of the internal behavior of the
   * device.
   *
   * <p>This constructor uses a single NEO as default with no load, however if you simulating the
   * motor using a different sim mechanism it can be ignored and setMotorCurrent() can be used
   * instead.
   *
   * @param spark The CANSparkBase to simulate.
   */
  public CANSparkSim1(CANSparkBase spark) {
    this(spark, SimDynamics.fromSim(new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00003973)));
  }

  /**
   * Get the simulated applied output. This matches the value from the CANSparkBase
   * getAppliedOutput(). Multiply by vbus to get the motor voltage.
   *
   * @return applied output [-1, 1]
   */
  public double getAppliedOutput() {
    return m_appliedOutput.get();
  }

  /**
   * Set the simulated applied output. Use this only in place of iterate().
   *
   * @param appliedOutput simulated applied output value [-1, 1]
   */
  public void setAppliedOutput(double appliedOutput) {
    m_appliedOutput.set(appliedOutput);
  }

  public double getSetpoint() {
    if (m_setpoint == null) {
      return 0.0;
    }
    return m_setpoint.get();
  }

  public double getArbFF() {
    if (m_arbFF == null) {
      return 0.0;
    }
    return m_arbFF.get();
  }

  public int getPidSlot() {
    if (m_pidSlot == null) {
      return 0;
    }
    return m_pidSlot.get();
  }

  // Modified from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
  private double runPID(double setpoint, double pv, int slot, double dt) {
    SparkPIDController controller = m_spark.getPIDController();
    double error = setpoint - pv;

    double p = error * controller.getP(slot);

    if (Math.abs(error) <= controller.getIZone(slot) || controller.getIZone(slot) != 0.0f) {
      m_iState = m_iState + (error * controller.getI(slot) * dt);
    } else {
      m_iState = 0;
    }

    double d = (error - m_prev_err) / dt;
    m_prev_err = error;
    d *= controller.getD(slot);

    double f = setpoint * controller.getFF(slot);

    double output = p + m_iState + d + f;
    m_pidOutput =
        Math.min(Math.max(output, controller.getOutputMin(slot)), controller.getOutputMax(slot));

    return m_pidOutput;
  }

  // return true if limited (i.e. output should be 0 in this direction)
  private boolean runLimitLogic(boolean forward) {
    if (forward) {
      if ((m_spark.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get())
          && CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(
              m_spark.sparkMaxHandle, SoftLimitDirection.kForward.value)) {
        return true;
      }

      return (CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(
              m_spark.sparkMaxHandle, SparkLimitSwitch.Direction.kForward.value)
          && m_forwardLimit);
    } else {
      if ((m_spark.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get())
          && CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(
              m_spark.sparkMaxHandle, SoftLimitDirection.kReverse.value)) {
        return true;
      }

      return (CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(
              m_spark.sparkMaxHandle, SparkLimitSwitch.Direction.kReverse.value)
          && m_reverseLimit);
    }
  }

  /**
   * Run internal calculations and set internal state when using simulated dynamics built in. If not
   * using the built in dynamics, call the three parameter version instead.
   *
   * @param vbus Bus voltage in volts
   * @param dt Simulation time step in seconds
   */
  public void iterate(double vbus, double dt) {
    iterate(m_simulatedDynamics.getAngularVelocityRPM(), vbus, dt);
    m_simulatedDynamics.setInputVoltage(m_spark.getAppliedOutput() * vbus);
    m_simulatedDynamics.update(dt);
    setMotorCurrent(m_simulatedDynamics.getCurrentDrawAmps());
  }

  private double m_iterateByPositionLast = 0.0;

  public void iterateByPosition(double position, double vbus, double dt) {
    // Non-ideal way to do this, but okay. Calculate the velocity here and force position
    iterate((position - m_iterateByPositionLast) * dt, vbus, dt);
    m_position.set(position);
    m_iterateByPositionLast = position;
  }

  /**
   * Run internal calculations and set internal state including
   *
   * <p>- Velocity (set by velocity, not calculated in this method) - Position - Bus Voltage (set by
   * vbus, not calculated in this method) - Current - Applied Output
   *
   * @param velocity The externally calculated velocity in units after conversion. For example, if
   *     the velocity factor is 1, use RPM. If the velocity factor is (1 / 60) use RPS. The internal
   *     simulation state will 'lag' behind this input due to the SPARK MAX internal filtering.
   *     Noise will also be added.
   * @param vbus Bus voltage in volts
   * @param dt Simulation time step in seconds
   */
  public void iterate(double velocity, double vbus, double dt) {
    // Velocity input is the system simulated input.
    double internalVelocity = NoiseGenerator.hallSensorVelocity(velocity);
    m_velocityAverage.put(internalVelocity, dt);
    internalVelocity = m_velocityAverage.get();

    assert vbus != 0.0;

    // First set the states that are given
    m_velocity.set(internalVelocity);

    double positionFactor =
        CANSparkMaxJNI.c_SparkMax_GetPositionConversionFactor(m_spark.sparkMaxHandle);
    double velocityFactor =
        CANSparkMaxJNI.c_SparkMax_GetVelocityConversionFactor(m_spark.sparkMaxHandle);

    assert positionFactor != 0.0;
    assert velocityFactor != 0.0;

    double velocityRPM = velocity / velocityFactor;
    m_position.set(m_position.get() + ((velocityRPM / 60) * dt) / positionFactor);
    m_busVoltage.set(vbus);

    // Calcuate the applied output
    double appliedOutput = 0.0;
    switch (m_controlMode.get()) {
        // Duty Cycle
      case 0:
        appliedOutput = getSetpoint();
        break;

        // Velocity
      case 1:
        appliedOutput = runPID(getSetpoint(), internalVelocity, getPidSlot(), dt);
        break;

        // Voltage
      case 2:
        appliedOutput = getSetpoint() / vbus;
        break;

        // Position
      case 3:
        appliedOutput = runPID(getSetpoint(), m_position.get(), getPidSlot(), dt);
        break;

        // Smart Motion
      case 4:
        // TODO... This control mechansim is not documented
        break;

        // Current
      case 5:
        appliedOutput = runPID(getSetpoint(), m_motorCurrent.get(), getPidSlot(), dt);
        break;

        // Smart Velocity
      case 6:
        // TODO... This control mechansim is not documented
        break;

      default:
        // REV Sim is borked and will end up here...
        // Logger.tag("CANSparkSim").error("Invalid control mode: {}", m_controlMode.get());
        break;
    }

    // ArbFF
    if (m_arbFFUnits == null || m_arbFFUnits.get() == 0) {
      // Voltage
      appliedOutput += getArbFF() / vbus;
    } else {
      // Duty Cycle
      appliedOutput += getArbFF();
    }

    // Limit to [-1, 1] or limit switch value
    double maxOutput = runLimitLogic(true) ? 0 : 1;
    double minOutput = runLimitLogic(false) ? 0 : -1;
    appliedOutput = Math.min(Math.max(appliedOutput, minOutput), maxOutput);

    // TODO: Voltage Comp

    // TODO: Selected Sensor?

    // TODO: Faults

    // And finally, set remaining states
    boolean doEnable = false;
    if (m_enable == null) {
      doEnable = DriverStation.isEnabled();
    } else {
      doEnable = m_enable;
    }

    if (doEnable) {
      m_appliedOutput.set(appliedOutput);
    } else {
      m_appliedOutput.set(0.0);
    }

    // TODO: Current Limits
  }

  /**
   * Get the simulation velocity. This should be equivilant to calling CANEncoder().getVelocity()
   *
   * @return Velocity of the SPARK MAX accounting for conversion factor
   */
  public double getVelocity() {
    return m_velocity.get();
  }

  /**
   * Set the simulation velocity. This method expects units after the conversion factor (your
   * program's native units).
   *
   * <p>Only use this method if not calling iterate()
   *
   * @param velocity simulation velocity
   */
  public void setVelocity(double velocity) {
    m_velocity.set(velocity);
  }

  /**
   * Get the simulation position. This should be equivilant to calling CANEncoder().getPosition()
   *
   * @return Velocity of the SPARK MAX
   */
  public double getPosition() {
    return m_position.get();
  }

  /**
   * Set the simulated position. This is equivilant to calling CANEncoder().setPosition(), in fact
   * you probably are using that unless you have a good reason to set the sim value separately, or
   * are running simulation without using iterate()
   *
   * @param position simulated position in your programs units (after conversion)
   */
  public void setPosition(double position) {
    m_position.set(position);
  }

  /**
   * Get the simulated bus voltage
   *
   * @return simulated bus voltage in volts
   */
  public double getBusVoltage() {
    return m_busVoltage.get();
  }

  /**
   * Set the simulated bus voltage. Use this if you are not using the iterate() method.
   *
   * @param voltage bus voltage in volts
   */
  public void setBusVoltage(double voltage) {
    m_busVoltage.set(voltage);
  }

  /**
   * Get the simulated motor current in amps. This is equivilant to running spark.getOutputCurrent()
   *
   * @return motor current in amps
   */
  public double getMotorCurrent() {
    return m_motorCurrent.get();
  }

  /**
   * Set the simulated motor current. The iterate() method also sets this value. If you are using an
   * external method to calculate the current, but still want to use the iterate() method, call this
   * function *after* iterate()
   *
   * @param current current in amps
   */
  public void setMotorCurrent(double current) {
    m_motorCurrent.set(current);
  }

  /**
   * Set the state of the forward limit switch. Set true to indicate that the forward limit switch
   * is set.
   *
   * <p>This method does not have any knowlege of polarity. So if you set the Spark limit switch as
   * 'normally open' and set tripped = true, then the limit is concidered closed (tripped). If you
   * set the Spark limit switch as 'normally closed' as set tripped = true, then the limit switch is
   * considered open (tripped).
   *
   * @param tripped set true to trip the forward limit
   */
  public void setForwardLimitSwitch(boolean tripped) {
    m_forwardLimit = tripped;
  }

  /**
   * Get the simulated forward limit switch state
   *
   * @return true if tripped. Does not look at whether or not enabled
   */
  public boolean getForwardLimitSwitch() {
    return m_forwardLimit;
  }

  /**
   * Set the state of the reverse limit switch. Set true to indicate that the reverse limit switch
   * is set.
   *
   * <p>This method does not have any knowlege of polarity. So if you set the Spark limit switch as
   * 'normally open' and set tripped = true, then the limit is concidered closed (tripped). If you
   * set the Spark limit switch as 'normally closed' as set tripped = true, then the limit switch is
   * considered open (tripped).
   *
   * @param tripped set true to trip the reverse limit
   */
  public void setReverseLimitSwitch(boolean tripped) {
    m_reverseLimit = tripped;
  }

  public void setFaults(CANSparkBase.FaultID... faults) {
    int result = 0;
    for (var fault : faults) {
      result |= (1 << fault.value);
    }
    m_faults.set(result);

    int stickyFaults = result | m_stickyFaults.get();
    m_stickyFaults.set(stickyFaults);
  }

  /**
   * Get the simulated reverse limit switch state
   *
   * @return true if tripped. Does not look at whether or not enabled
   */
  public boolean getReverseLimitSwitch() {
    return m_reverseLimit;
  }

  public void enable() {
    m_enable = true;
  }

  public void disable() {
    m_enable = false;
  }

  public void useDriverStationEnable() {
    m_enable = null;
  }
}
