package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkBaseHandle;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.controller.PIDGains;
import frc.lib.electromechanical.Encoder;
import frc.lib.electromechanical.EncoderSupplier;
import frc.lib.monitor.HealthMonitor;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.util.AccessOrderHashSet;
import frc.lib.util.Constraints;
import frc.lib.util.Faults;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;
import org.tinylog.Logger;

public class SparkFlex extends CANSparkFlex implements TelemetryNode {

  // To be removed
  private AccessOrderHashSet<BiFunction<CANSparkFlex, Boolean, Boolean>> m_mutatorChain;
  private AccessOrderHashSet<Function<Boolean, Boolean>> m_settingsFcns;

  private List<SparkFlex> m_followers = new ArrayList<>();
  private final int kParameterSetAttemptCount = 5;
  private final CANSparkBaseHandle m_sparkBaseHandle;
  private final String m_name;
  private SparkMaxLogger m_logger = null;

  private static final int kStatus0FollowerRateMs = 10;
  private static final int kDefaultCANTimeout = 50;

  /**
   * Store a reference to every spark max.
   *
   * <p>TODO: Release the reference if it is closed.
   */
  private static List<SparkFlex> m_sparkMaxes = new ArrayList<>();

  private static int m_burnFlashCnt = 0;

  private static SparkMonitor s_monitor = new SparkMonitor();

  /**
   * Reinitialize the SparkMax by running through all mutations on the object in order.
   *
   * @return true if reinitialized correctly
   */
  private boolean reinitFunction() {
    Logger.tag("SparkMax")
        .debug("===Reinitializing!! SparkMax with Id {} ({})===", getDeviceId(), m_name);
    // To be removed
    for (var fcn : m_mutatorChain) {
      if (!fcn.apply(this, false)) {
        return false;
      }
    }
    for (var fcn : m_settingsFcns) {
      if (!fcn.apply(false)) {
        return false;
      }
    }
    return clearFaults() == REVLibError.kOk;
  }

  /**
   * Create a motor controller from a CANSparkFlex object.
   *
   * @param spark A CANSparkFlex object. Run any initialization that you want excluded from the
   *     built in health monitor functions
   * @param initFunction A function which takes a CANSparkFlex and returns a Boolean. This function
   *     is used to initialize the CANSparkFlex device, and is called in one of two places. 1) It is
   *     called in this constructor, and 2) it is called in the case of a health monitor timeout
   *     (i.e. the controller has reset)
   * @param name assign a name to this controller
   */
  public SparkFlex(int canId, MotorType motorType, String name) {
    super(canId, motorType);
    m_name = name;

    // Always start fresh and apply settings in code for each device
    // Add delay to avoid any possible timing issues.
    restoreFactoryDefaults();
    Timer.delay(0.050);

    setCANTimeout(kDefaultCANTimeout);

    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus3, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus4, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus7, 30000);

    // Steal the private handle to poke around the internals
    m_sparkBaseHandle = new CANSparkBaseHandle(this);

    m_mutatorChain = new AccessOrderHashSet<>();
    m_settingsFcns = new AccessOrderHashSet<>();
    HealthMonitor.monitor(() -> this.getStickyFault(FaultID.kHasReset), () -> reinitFunction());
    m_sparkMaxes.add(this);
    Logger.tag("SparkMax").debug("Initializing SparkMax with Id {} ({})", canId, m_name);
    if (m_burnFlashCnt > 0) {
      Logger.tag("SparkMax")
          .warn(
              "SparkMax with ID {} initialized after burning flash, flash count {}",
              canId,
              m_burnFlashCnt);
    }
    s_monitor.add(this);
  }

  /**
   * Create a SPARK MAX motor controller with a brushless motor
   *
   * @param canId Spark Max CAN Id
   */
  public SparkFlex(int canId) {
    this(canId, MotorType.kBrushless, "Spark-" + String.valueOf(canId));
  }

  public SparkFlex(int canId, MotorType motorType) {
    this(canId, motorType, "Spark-" + String.valueOf(canId));
  }

  public SparkFlex(int canId, String name) {
    this(canId, MotorType.kBrushless, name);
  }

  /**
   * Create spark max object with an initializer method. This method is called on initialization as
   * well as if the spark max resets.
   *
   * @param initialize function that returns true on success, that takes a CANSparkFlex and a
   *     Boolean to indicate if the call is from initialization or after a reset.
   * @return this
   */
  public SparkFlex withInitializer(BiFunction<CANSparkFlex, Boolean, Boolean> initialize) {
    m_mutatorChain.add(initialize);

    if (Robot.isSimulation()) {
      initialize.apply(this, true);
      return this;
    }

    int setAttemptNumber = 0;
    while (initialize.apply(this, true) != true) {
      Logger.tag("Spark Max")
          .warn(
              "Spark Max ID {} ({}): Failed to initialize, attempt {} of {}",
              getDeviceId(),
              m_name,
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max")
            .error("Spark Max ID {} ({}): Failed to initialize!!", getDeviceId(), m_name);
        Faults.subsystem("SparkMax").fatal("Initialize for SPARK MAX: " + m_name);
        break;
      }
    }

    return this;
  }

  public SparkFlex initialize(Function<Boolean, Boolean> initializer) {
    m_settingsFcns.add(initializer);

    if (Robot.isSimulation()) {
      initializer.apply(true);
      return this;
    }

    int setAttemptNumber = 0;
    while (initializer.apply(true) != true) {
      Logger.tag("Spark Max")
          .warn(
              "Spark Max ID {} ({}): Failed to initialize, attempt {} of {}",
              getDeviceId(),
              m_name,
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max")
            .error("Spark Max ID {} ({}): Failed to initialize!!", getDeviceId(), m_name);
        Faults.subsystem("SparkMax").fatal("Initialize for SPARK MAX: " + m_name);
        break;
      }
    }

    return this;
  }

  /**
   * Set some parameter, fire and forget. Values here will not be reset if the controller loses
   * power. Results cannot be checked.
   *
   * @param fcn function with only parameter calls, should not block or any time.
   */
  public void fastParameterSet(Runnable fcn) {
    setCANTimeout(0);
    fcn.run();
    setCANTimeout(kDefaultCANTimeout);
  }

  // public SparkMaxLogger getLogger(DataLog log) {
  //   if (m_logger == null) {
  //     m_logger = new SparkMaxLogger(this, m_name, log);
  //   }
  //   return m_logger;
  // }

  // public SparkMaxLogger getLogger() {
  //   if (m_logger == null) {
  //     m_logger = getLogger(DataLogManager.getLog());
  //   }
  //   return m_logger;
  // }

  /**
   * Create a Controller based on the internal spark max velocity controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @return
   */
  public SparkController velocityController(PIDGains gains) {
    return new SparkController(this, getPIDController(), ControlType.kVelocity, gains, -1.0, 1.0);
  }

  /**
   * Create a Controller based on the internal spark max velocity controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @param minOutput the minimum output for the controller in percent [-1, 1) must be lower than
   *     max
   * @param maxOutput the max output of the controller in percent (-1, 1] must be higher than min
   * @return
   */
  public SparkController velocityController(PIDGains gains, double minOutput, double maxOutput) {
    return new SparkController(
        this, getPIDController(), ControlType.kVelocity, gains, minOutput, maxOutput);
  }

  /**
   * Create a Controller based on the internal spark max position controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @return
   */
  public SparkController positionController(PIDGains gains) {
    return new SparkController(this, getPIDController(), ControlType.kPosition, gains, -1.0, 1.0);
  }

  /**
   * Create a Controller based on the internal spark max position controller using the duty cycle
   * sensor
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @return
   */
  public SparkController positionController(PIDGains gains, MotorFeedbackSensor sensor) {
    var controller = getPIDController();
    controller.setFeedbackDevice(sensor);
    return new SparkController(this, controller, ControlType.kPosition, gains, -1.0, 1.0);
  }

  /**
   * Create a profiled PID controller based on the spark max smart motion
   *
   * @param gains
   * @param constraints
   * @return
   */
  public SparkController profiledController(PIDGains gains, Constraints constraints) {
    mutate(
        (spark, firstCall) -> {
          int errors = 0;
          errors +=
              SparkMaxUtils.check(
                  spark
                      .getPIDController()
                      .setSmartMotionMaxAccel(
                          constraints.maxAcceleration, SparkController.SMART_MOTION_GAIN_SLOT));
          errors +=
              SparkMaxUtils.check(
                  spark
                      .getPIDController()
                      .setSmartMotionMaxVelocity(
                          constraints.maxVelocity, SparkController.SMART_MOTION_GAIN_SLOT));
          return errors == 0;
        });
    return new SparkController(this, getPIDController(), ControlType.kPosition, gains, -1.0, 1.0);
  }

  /**
   * Create an encoder object based on the Spark Max internal encoder
   *
   * @return Encoder object
   */
  public Encoder builtinEncoder() {
    return new EncoderSupplier(
        () -> getEncoder().getVelocity(),
        () -> getEncoder().getPosition(),
        pos -> getEncoder().setPosition(pos));
  }

  /**
   * Modify CANSparkFlex object. Mutations using this method are re-run in order in the case of a
   * device failure that is later recovered. The same function can be called multiple times, and
   * will simply be moved to the end of the list each call.
   *
   * <p>Only adds the function to the list if it succeeds
   *
   * @param fcn a function on the underlying CANSparkFlex object returning true on success.
   *     Typically used to change parameter values. Function should run quickly and return.
   * @return result of mutate function
   */
  public boolean mutate(BiFunction<CANSparkFlex, Boolean, Boolean> fcn) {
    Boolean result = fcn.apply(this, true);

    if (m_burnFlashCnt > 0) {
      Logger.tag("SparkMax Controller")
          .warn(
              "Running mutate function after burn flash. CanID {}, FlashCnt {}",
              getDeviceId(),
              m_burnFlashCnt);
    }

    int setAttemptNumber = 0;
    while (result == null || result != true) {
      Logger.tag("Spark Max")
          .warn(
              "Spark Max ID {} ({}): Failed to run mutator, attempt {} of {}",
              getDeviceId(),
              m_name,
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max")
            .error("Spark Max ID {}: Failed to run mutator function!!", getDeviceId());
        break;
      }
      result = fcn.apply(this, true);
    }

    if (result != null && result) {
      m_mutatorChain.add(fcn);
    }
    return result == null ? false : result;
  }

  public boolean setterFunction(Function<Boolean, Boolean> fcn) {
    Boolean result = fcn.apply(true);

    if (m_burnFlashCnt > 0) {
      Logger.tag("SparkMax Controller")
          .warn(
              "Running setter function after burn flash. CanID {}, FlashCnt {}",
              getDeviceId(),
              m_burnFlashCnt);
    }

    int setAttemptNumber = 0;
    while (result == null || result != true) {
      Logger.tag("Spark")
          .warn(
              "Spark ID {} ({}): Failed to run setter, attempt {} of {}",
              getDeviceId(),
              m_name,
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max")
            .error("Spark Max ID {}: Failed to run setter function!!", getDeviceId());
        break;
      }
      result = fcn.apply(true);
    }

    if (result != null && result) {
      m_settingsFcns.add(fcn);
    }
    return result == null ? false : result;
  }

  public enum FrameStrategy {
    kDefault,
    kVelocity,
    kPosition,
    kNoFeedback,
    kCurrent,
    kVoltage,
    kVelocityAndPosition,
  }

  /**
   * Set a frame strategy for the feedback frames. This changes the periodic frame rates to be
   * either slow (500ms) or fast (15ms) depending on the signal needed.
   *
   * @param spark CANSparkFlex object to run this on
   * @param strategy Signal strategy
   * @param withFollower does this device have a follower motor
   */
  public void setFrameStrategy(FrameStrategy strategy) {
    final int slowFrame = 500;
    final int fastFrame = 15;
    boolean hasFollower = !m_followers.isEmpty();
    int status0 = 10;
    int status1 = 20;
    int status2 = 20;
    int status3 = 50;
    switch (strategy) {
      case kVelocity:
      case kCurrent:
      case kVoltage:
        status1 = fastFrame;
        status2 = slowFrame;
        status3 = slowFrame;
        break;
      case kPosition:
        status1 = slowFrame;
        status2 = fastFrame;
        status3 = slowFrame;
        break;
      case kNoFeedback:
        status1 = slowFrame;
        status2 = slowFrame;
        status3 = slowFrame;
        break;
      case kVelocityAndPosition:
        status1 = fastFrame;
        status2 = fastFrame;
        status3 = slowFrame;
        break;
      case kDefault:
      default:
        // already set
        break;
    }
    if (hasFollower) {
      status0 = kStatus0FollowerRateMs;
    } else {
      // Lets get this a little faster since it has
      // errors and appiled output
      status0 = 100;
    }

    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus0, status0);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus1, status1);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus2, status2);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus3, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus4, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus5, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus6, 30000);
    SparkMaxUtils.setPeriodicFramePeriod(this, PeriodicFrame.kStatus7, 30000);
  }

  public SparkFlex withFollower(SparkFlex follower) {
    return withFollower(follower, false);
  }

  public SparkFlex withFollower(SparkFlex follower, boolean invert) {
    mutate(
        (spark, firstCall) -> {
          // Always set this when there is a follower
          return SparkMaxUtils.setPeriodicFramePeriod(
                  spark, PeriodicFrame.kStatus0, kStatus0FollowerRateMs)
              == REVLibError.kOk;
        });

    follower.mutate(
        (spark, firstCall) -> {
          return spark.follow(this, invert) == REVLibError.kOk;
        });
    m_followers.add(follower);
    return this;
  }

  public String getName() {
    return m_name;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.addStringProperty("Name", this::getName, null);
    builder.addIntegerProperty("ID", this::getDeviceId, null);
    builder.addDoubleProperty("Current", this::getOutputCurrent, null);
    builder.addDoubleProperty("Applied Output", this::getAppliedOutput, null);
  }

  /**
   * Run burnFlash() for all controllers initialized. The ideal use case for this call is to call it
   * once everything has been initialized. The burnFlash() call has the side effect of preventing
   * all communication *to* the device for up to 200ms or more, potentially including some messages
   * called before the burnFlash() call, and receiveing messages *from* the device.
   *
   * <p>WARNING: This call will sleep the thread before and after burning flash. This is for your
   * safety.
   */
  public static void burnFlashInSync() {
    Logger.tag("SparkMax").debug("Burning Flash Count: {}", ++m_burnFlashCnt);
    Timer.delay(0.25);
    for (SparkFlex max : m_sparkMaxes) {
      Logger.tag("SparkMax").trace("Burning flash for Can ID {}", max.getDeviceId());
      max.burnFlash();
      // Enough time to not spam the bus too bad
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
    Logger.tag("SparkMax").debug("Burn Flash Complete.");
  }

  public static Map<Integer, String> getNameMap() {
    HashMap<Integer, String> result = new HashMap<>();
    for (SparkFlex flex : m_sparkMaxes) {
      result.put(flex.getDeviceId(), flex.getName());
    }
    return result;
  }
}
