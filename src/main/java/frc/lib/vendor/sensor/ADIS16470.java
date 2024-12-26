/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * This package is started from https://github.com/juchong/ADIS16470-RoboRIO-Driver commit
 * bcb6ed6ac4d323bc329bdd1b65342f6792dd1256
 *
 * <p>Major changes from the above:
 *
 * <p>- Only pull data relevent to yaw (removes complimentary filter and inclinometer) - Remove all
 * threading since it is not actually needed, assuming the user calls the update function frequently
 * enough - Adds an update function to run the gyro calculatuon instead of a thread - Removes some
 * of the configuration API since it makes no sense to changes these after initializaiton (e.g.
 * setting a new yaw axis) - Changes the initialization and calibration functions to the following
 * steps:
 *
 * <p>1) Initialize everything but do not start auto-spi or calibrate 2) Allow user to call a new
 * start() function which kicks off the calibration and starts auto-spi 3) Sensor config is now in
 * an immutable state, unless the sensor suffers a major power incident. This means the sensor no
 * longer changes between auto and standard SPI modes.
 *
 * <p>The motivation here is to simplify the code after initialization, and to allow longer
 * initialization times. This matches the way FRC events are conducted. The user does not call
 * start() on the sensor until the beginning of auto. The sensor automatically uses the *past* data
 * to calibrate, and after only a 50ms delay the sensor is ready to go. This is perfect for the FRC
 * use case, since the robot is typically on the field sitting idle for a good amount of time before
 * auton begins. The user no longer needs to worry about not touching the robot when first powering
 * on, or setting code etc. and the calibration time can be longer.
 *
 * <p>To use this driver:
 *
 * <p>1) Instantiate the class 2) Call run() after the calibration time has elapsed, typically when
 * the robot is first enabled
 */
package frc.lib.vendor.sensor;

// import java.lang.FdLibm.Pow;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AsynchronousInterrupt2;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import org.tinylog.Logger;

/** This class is for the ADIS16470 IMU that connects to the RoboRIO SPI port. */
@SuppressWarnings("unused")
public class ADIS16470 implements TelemetryNode, AutoCloseable {

  /* ADIS16470 Register Map Declaration */
  private static final int FLASH_CNT = 0x00; // Flash memory write count
  private static final int DIAG_STAT = 0x02; // Diagnostic and operational status
  private static final int X_GYRO_LOW = 0x04; // X-axis gyroscope output, lower word
  private static final int X_GYRO_OUT = 0x06; // X-axis gyroscope output, upper word
  private static final int Y_GYRO_LOW = 0x08; // Y-axis gyroscope output, lower word
  private static final int Y_GYRO_OUT = 0x0A; // Y-axis gyroscope output, upper word
  private static final int Z_GYRO_LOW = 0x0C; // Z-axis gyroscope output, lower word
  private static final int Z_GYRO_OUT = 0x0E; // Z-axis gyroscope output, upper word
  private static final int X_ACCL_LOW = 0x10; // X-axis accelerometer output, lower word
  private static final int X_ACCL_OUT = 0x12; // X-axis accelerometer output, upper word
  private static final int Y_ACCL_LOW = 0x14; // Y-axis accelerometer output, lower word
  private static final int Y_ACCL_OUT = 0x16; // Y-axis accelerometer output, upper word
  private static final int Z_ACCL_LOW = 0x18; // Z-axis accelerometer output, lower word
  private static final int Z_ACCL_OUT = 0x1A; // Z-axis accelerometer output, upper word
  private static final int TEMP_OUT = 0x1C; // Temperature output (internal, not calibrated)
  private static final int TIME_STAMP = 0x1E; // PPS mode time stamp
  private static final int DATA_CNTR = 0x22; // Data counter
  private static final int X_DELTANG_LOW = 0x24; // X-axis delta angle output, lower word
  private static final int X_DELTANG_OUT = 0x26; // X-axis delta angle output, upper word
  private static final int Y_DELTANG_LOW = 0x28; // Y-axis delta angle output, lower word
  private static final int Y_DELTANG_OUT = 0x2A; // Y-axis delta angle output, upper word
  private static final int Z_DELTANG_LOW = 0x2C; // Z-axis delta angle output, lower word
  private static final int Z_DELTANG_OUT = 0x2E; // Z-axis delta angle output, upper word
  private static final int X_DELTVEL_LOW = 0x30; // X-axis delta velocity output, lower word
  private static final int X_DELTVEL_OUT = 0x32; // X-axis delta velocity output, upper word
  private static final int Y_DELTVEL_LOW = 0x34; // Y-axis delta velocity output, lower word
  private static final int Y_DELTVEL_OUT = 0x36; // Y-axis delta velocity output, upper word
  private static final int Z_DELTVEL_LOW = 0x38; // Z-axis delta velocity output, lower word
  private static final int Z_DELTVEL_OUT = 0x3A; // Z-axis delta velocity output, upper word
  private static final int XG_BIAS_LOW =
      0x40; // X-axis gyroscope bias offset correction, lower word
  private static final int XG_BIAS_HIGH =
      0x42; // X-axis gyroscope bias offset correction, upper word
  private static final int YG_BIAS_LOW =
      0x44; // Y-axis gyroscope bias offset correction, lower word
  private static final int YG_BIAS_HIGH =
      0x46; // Y-axis gyroscope bias offset correction, upper word
  private static final int ZG_BIAS_LOW =
      0x48; // Z-axis gyroscope bias offset correction, lower word
  private static final int ZG_BIAS_HIGH =
      0x4A; // Z-axis gyroscope bias offset correction, upper word
  private static final int XA_BIAS_LOW =
      0x4C; // X-axis accelerometer bias offset correction, lower word
  private static final int XA_BIAS_HIGH =
      0x4E; // X-axis accelerometer bias offset correction, upper word
  private static final int YA_BIAS_LOW =
      0x50; // Y-axis accelerometer bias offset correction, lower word
  private static final int YA_BIAS_HIGH =
      0x52; // Y-axis accelerometer bias offset correction, upper word
  private static final int ZA_BIAS_LOW =
      0x54; // Z-axis accelerometer bias offset correction, lower word
  private static final int ZA_BIAS_HIGH =
      0x56; // Z-axis accelerometer bias offset correction, upper word
  private static final int FILT_CTRL = 0x5C; // Filter control
  private static final int MSC_CTRL = 0x60; // Miscellaneous control
  private static final int UP_SCALE = 0x62; // Clock scale factor, PPS mode
  private static final int DEC_RATE = 0x64; // Decimation rate control (output data rate)
  private static final int NULL_CNFG = 0x66; // Auto-null configuration control
  private static final int GLOB_CMD = 0x68; // Global commands
  private static final int FIRM_REV = 0x6C; // Firmware revision
  private static final int FIRM_DM = 0x6E; // Firmware revision date, month and day
  private static final int FIRM_Y = 0x70; // Firmware revision date, year
  private static final int PROD_ID = 0x72; // Product identification
  private static final int SERIAL_NUM = 0x74; // Serial number (relative to assembly lot)
  private static final int USER_SCR1 = 0x76; // User scratch register 1
  private static final int USER_SCR2 = 0x78; // User scratch register 2
  private static final int USER_SCR3 = 0x7A; // User scratch register 3
  private static final int FLSHCNT_LOW = 0x7C; // Flash update count, lower word
  private static final int FLSHCNT_HIGH = 0x7E; // Flash update count, upper word

  public enum IMUAxis {
    kX,
    kY,
    kZ
  }

  public enum ADIS16470CalibrationTime {
    _32ms(0),
    _64ms(1),
    _128ms(2),
    _256ms(3),
    _512ms(4),
    _1s(5),
    _2s(6),
    _4s(7),
    _8s(8),
    _16s(9),
    _32s(10),
    _64s(11);
    private int value;
    private static final double[] times = {
      0.032, 0.064, 0.128, 0.256, 0.512, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0
    };

    private ADIS16470CalibrationTime(int value) {
      this.value = value;
    }

    public double toSeconds() {
      return times[value];
    }

    /**
     * Create the constant from a time delta using the best time available
     *
     * @param seconds time in seconds
     * @return closest calibration time without going over
     */
    public static ADIS16470CalibrationTime fromTimeDelta(double seconds) {
      for (int i = 1; i < times.length; i++) {
        if (times[i] > seconds) {
          return ADIS16470CalibrationTime.values()[i - 1];
        }
      }
      return _32ms;
    }
  }

  // Static Constants
  private static final double delta_angle_sf = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  private static final double rad_to_deg = 57.2957795;
  private static final double deg_to_rad = 0.0174532;

  // Integrated gyro angle
  private double m_integ_angle = 0.0;
  private double m_y_integ_angle = 0.0;
  private double m_y_gyro_raw = 0.0;
  private double m_accelX = 0.0;
  private double m_accelY = 0.0;
  private double m_accelZ = 0.0;
  private final double kFilterTimeConstant = 0.25;
  private LinearFilter m_accelXFilter = LinearFilter.singlePoleIIR(kFilterTimeConstant, 0.01);
  private LinearFilter m_accelYFilter = LinearFilter.singlePoleIIR(kFilterTimeConstant, 0.01);
  private LinearFilter m_accelZFilter = LinearFilter.singlePoleIIR(kFilterTimeConstant, 0.01);

  // State variables
  private final ADIS16470CalibrationTime m_calibration_time;
  private int m_threadRunCnt = 0;
  private double m_scaled_sample_rate = 2500.0;
  private boolean m_auto_configured = false;
  private final double m_startTime;
  private int m_dataCount;
  private AtomicInteger m_dataCountErrors = new AtomicInteger();
  private AtomicInteger m_otherErrorCount = new AtomicInteger();

  // Resources
  private SPI m_spi;
  private SPI.Port m_spi_port;
  private DigitalInput m_auto_interrupt;
  private DigitalOutput m_reset_out;
  private DigitalInput m_reset_in;
  private DigitalOutput m_status_led;
  private ByteBuffer m_rwbuf = ByteBuffer.allocateDirect(2);

  // Interrupt
  private final AsynchronousInterrupt2 m_interrupt;
  private final ReentrantLock m_processLock = new ReentrantLock();

  // Sim
  private SimDevice m_simDevice;
  private SimDouble m_simAngle;

  // Previous timestamp
  long previous_timestamp = 0;

  /**
   * @param yaw_axis Which axis is Yaw
   * @param port SPI port to use
   * @throws Exception
   */
  public ADIS16470(ADIS16470CalibrationTime cal_time) {
    m_calibration_time = cal_time;
    m_spi_port = SPI.Port.kOnboardCS0;

    // Simulation
    m_simDevice = SimDevice.create("Gyro:ADIS16470", m_spi_port.value);
    if (m_simDevice != null) {
      m_simAngle = m_simDevice.createDouble("angle_x", SimDevice.Direction.kInput, 0.0);
    }

    if (m_simDevice == null) {
      // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
      // Relies on the RIO hardware by default configuring an output as low
      // and configuring an input as high Z. The 10k pull-up resistor internal to the
      // IMU then forces the reset line high for normal operation.
      m_reset_out = new DigitalOutput(27); // Drive SPI CS2 (IMU RST) low
      m_reset_out.set(false);
      Timer.delay(0.01);
      m_reset_out.set(true);
      // m_reset_out.close();
      // m_reset_in = new DigitalInput(27); // Set SPI CS2 (IMU RST) high
      Timer.delay(0.25); // Wait 250ms for reset to complete
      m_startTime = Timer.getFPGATimestamp();

      // Configure Data-Ready pin on SPI CS1
      m_auto_interrupt = new DigitalInput(26);

      for (int i = 0; !initializeSPI(); i++) {
        // Retry in 1 second
        Logger.tag("ADIS16470").warn("Unable to initialize device, attempt {}", i);
        m_otherErrorCount.incrementAndGet();
        Timer.delay(1);

        if (i == 5) {
          Logger.tag("ADIS16470").error("Unable to initialize device");
          m_interrupt = null;
          return;
        }
      }

      // Set IMU internal decimation to 100 SPS (output data rate of 2000 SPS / (19 + 1) = 100Hz)
      writeRegister(DEC_RATE, 0x13);

      // Set data ready polarity (raising), Disable gSense Compensation and PoP
      writeRegister(MSC_CTRL, 0xC1);

      // Disable IMU internal Bartlett filter
      writeRegister(FILT_CTRL, 2);

      // Run a continuous bias calibration time based on fast setting in case
      // user setting does not have enough data.
      writeRegister(NULL_CNFG, (ADIS16470CalibrationTime._1s.value | 0x0700));

      // TODO: Run this in the thread instead
      Timer.delay(1.100); // Wait 1s + margin
      writeRegister(GLOB_CMD, 0x1); // calibrate
      Timer.delay(0.05); // Wait 50ms GLOB_CMD timing

      // Configure continuous bias calibration time based on user setting
      writeRegister(NULL_CNFG, (m_calibration_time.value | 0x0700));

      // Drive "Ready" LED to 'waiting to start' state
      m_status_led = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low
      // Only 1 PWM frequency for the entire set of DIO, so this may need to change...
      m_status_led.set(true);

      // Interrupt Settings
      m_interrupt = new AsynchronousInterrupt2(m_auto_interrupt, this::acquire);
      m_interrupt.setInterruptEdges(true, false);
      m_interrupt.enable();

      // Let the user know the IMU was initiallized successfully
      Logger.tag("ADIS16470").debug("Successfully Initialized, waiting to start...");
    } else {
      m_startTime = Timer.getFPGATimestamp();
      m_interrupt = null;
      m_status_led = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low
      m_status_led.set(true);
    }

    // Report usage and post data to DS
    HAL.report(tResourceType.kResourceType_ADIS16470, 0);
  }

  public ADIS16470() {
    this(ADIS16470CalibrationTime._16s);
  }

  public void calibrate() {
    double startDelay = Timer.getFPGATimestamp() - m_startTime;
    boolean skipCal = false;

    if (startDelay < m_calibration_time.toSeconds()) {
      skipCal = true;
      Logger.tag("ADIS16470")
          .warn(
              "Calibration time too short! Code is run for {}s calibration is set to {}. Using previous calibration of 1s",
              startDelay,
              m_calibration_time.toSeconds());
    }

    if (m_simDevice == null) {
      if (!skipCal) {
        // Write offset calibration command to IMU
        // The SPI driver is thread safe
        int i = 0;
        for (i = 0; i < 5; i++) {
          if (writeRegister(GLOB_CMD, 0x1)) {
            break;
          }
          Timer.delay(0.002);
        }
        Logger.tag("ADIS16470").debug("GLOB CMD ADIS16470 runs: {}", i);
        // Timer.delay(0.05); // Wait 50ms GLOB_CMD timing
      }

      Logger.tag("ADIS16470").debug("Started ADIS16470");
    } else {
      Logger.tag("ADIS16470").debug("Started Sim ADIS16470");
    }

    m_status_led.set(false);
    Logger.tag("ADIS16470")
        .info("Calibration took {} seconds", Timer.getFPGATimestamp() - (startDelay + m_startTime));
  }

  /**
   * @param buf
   * @return
   */
  private static int toUShort(ByteBuffer buf) {
    return (buf.getShort(0)) & 0xFFFF;
  }

  /**
   * @param buf
   * @return
   */
  private static int toUShort(byte[] buf) {
    return (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }

  /**
   * @param data
   * @return
   */
  private static int toUShort(int... data) {
    byte[] buf = new byte[data.length];
    for (int i = 0; i < data.length; ++i) {
      buf[i] = (byte) data[i];
    }
    return toUShort(buf);
  }

  /**
   * @param sint
   * @return
   */
  private static long toULong(int sint) {
    return sint & 0x00000000FFFFFFFFL;
  }

  /**
   * @param buf
   * @return
   */
  private static int toShort(int... buf) {
    return (short) (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }

  /**
   * @param buf
   * @return
   */
  private static int toShort(ByteBuffer buf) {
    return toShort(buf.get(0), buf.get(1));
  }

  /**
   * @param buf
   * @return
   */
  private static int toShort(byte[] buf) {
    return buf[0] << 8 | buf[1];
  }

  /**
   * @param buf
   * @return
   */
  private static int toInt(int... buf) {
    return (int)
        ((buf[0] & 0xFF) << 24 | (buf[1] & 0xFF) << 16 | (buf[2] & 0xFF) << 8 | (buf[3] & 0xFF));
  }

  /**
   * Get the SPI port number.
   *
   * @return The SPI port number.
   */
  public int getPort() {
    return m_spi_port.value;
  }

  /**
   * Initialize the SPI port
   *
   * @return true if the port initialized successfully
   */
  private boolean initializeSPI() {
    if (m_spi == null) {
      Logger.tag("ADIS16470").debug("Initializing SPI port.");
      m_spi = new SPI(m_spi_port);
    } else {
      Logger.tag("ADIS16470").debug("Re-initializing SPI port.");
      m_spi.close();
      m_spi = new SPI(m_spi_port);
    }

    m_spi.setClockRate(2000000);
    m_spi.setMode(SPI.Mode.kMode3);
    m_spi.setChipSelectActiveLow();
    readRegister(PROD_ID); // Dummy read (but why??)

    // Validate the product ID
    if (readRegister(PROD_ID) != 16982) {
      Logger.tag("ADIS16470").error("Could not find ADIS16470");
      m_otherErrorCount.incrementAndGet();
      m_spi.close();
      m_spi = null;
      return false;
    }

    return true;
  }

  private AtomicInteger m_readRegisterErrors = new AtomicInteger(0);
  private AtomicInteger m_writeRegisterErrors = new AtomicInteger(0);

  /**
   * @param reg
   * @return
   */
  private int readRegister(int reg) {
    m_rwbuf.order(ByteOrder.BIG_ENDIAN);
    m_rwbuf.put(0, (byte) (reg & 0x7f));
    m_rwbuf.put(1, (byte) 0);

    if (m_spi.write(m_rwbuf, 2) < 0) {
      m_readRegisterErrors.incrementAndGet();
    }

    if (m_spi.read(false, m_rwbuf, 2) < 0) {
      m_readRegisterErrors.incrementAndGet();
    }

    return toUShort(m_rwbuf);
  }

  /**
   * @param reg
   * @param val
   */
  private boolean writeRegister(int reg, int val) {
    boolean result = true;
    // low byte
    m_rwbuf.put(0, (byte) ((0x80 | reg)));
    m_rwbuf.put(1, (byte) (val & 0xff));

    if (m_spi.write(m_rwbuf, 2) < 0) {
      m_writeRegisterErrors.incrementAndGet();
      result = false;
    }

    // high byte
    m_rwbuf.put(0, (byte) (0x81 | reg));
    m_rwbuf.put(1, (byte) (val >> 8));
    if (m_spi.write(m_rwbuf, 2) < 0) {
      m_writeRegisterErrors.incrementAndGet();
      result = false;
    }

    return result;
  }

  /** {@inheritDoc} */
  public void reset() {
    m_processLock.lock();
    m_integ_angle = 0.0;
    m_y_integ_angle = 0.0;
    m_processLock.unlock();
  }

  /** Delete (free) the spi port used for the IMU. */
  @Override
  public void close() {
    Logger.tag("ADIS16470").info("Closing Device.");
    if (m_interrupt != null) {
      m_interrupt.disable();
      m_interrupt.close();
    }
    if (m_spi != null) {
      m_spi.close();
      if (m_auto_interrupt != null) {
        m_auto_interrupt.close();
        m_auto_interrupt = null;
      }
      m_spi = null;
    }
  }

  /** Called in the interrupt context, which is a high priority real-time thread. */
  private void acquire(boolean rising, boolean falling) {
    // TODO: This probably doesn't ever actually get called in sim...
    if (m_simDevice != null) {
      m_processLock.lock();
      m_integ_angle = m_simAngle.get();
      m_processLock.unlock();
      return;
    }

    long timestamp = (long) (m_interrupt.getRisingTimestamp() * 1e6);

    long dt_us = timestamp - previous_timestamp;
    int deltaAngle = readRegister(Z_DELTANG_LOW) | (readRegister(Z_DELTANG_OUT) << 16);
    int yAngle = readRegister(Y_DELTANG_LOW) | (readRegister(Y_DELTANG_OUT) << 16);
    double accelX = ((short) readRegister(X_ACCL_OUT)) / 800.0;
    double accelY = ((short) readRegister(Y_ACCL_OUT)) / 800.0;
    double accelZ = 0.0;
    double rawGyroY = ((short) readRegister(X_GYRO_OUT)) * -1.0;
    int dataCount = readRegister(DATA_CNTR);
    int status = 0;

    accelX = m_accelXFilter.calculate(accelX);
    accelY = m_accelYFilter.calculate(accelY);

    /* Get delta angle value for selected yaw axis and scale by the elapsed time (based on timestamp) */
    double scalar = 1.005828380585E-10;
    double delta_angle = ((double) deltaAngle * scalar) / (1.0 / (double) (dt_us));
    double y_delta_angle = ((double) yAngle * scalar) / (1.0 / (double) (dt_us));

    if (m_threadRunCnt < 500) {
      // Throw our first few samples which is too stale
      m_processLock.lock();
      m_integ_angle = 0.0;
      y_delta_angle = 0.0;
      m_processLock.unlock();
      m_dataCount = dataCount;
      m_threadRunCnt++;
    } else {
      boolean skipIntegration = false;
      if (previous_timestamp > timestamp || (dt_us) > 11000 || (dt_us) < 9000) {
        if (previous_timestamp != 0) {
          m_otherErrorCount.incrementAndGet();
          /*
           * This call takes too long, look at the count increment instead
          Logger.tag("ADIS16470")
              .error(
                  "Timestamp Error! Prev: {} Current: {} Delta: {} Delta Angle: {}",
                  previous_timestamp,
                  timestamp,
                  dt_us,
                  delta_angle);
                  */
        }
        skipIntegration = true;
      }

      if (dataCount != ((m_dataCount + 1) & 0xFFFF)) {
        /*
        * This call takes too long, look at the count increment instead
        Logger.tag("ADIS16470")
            .error("Data count failed! Expected {} got {}", m_dataCount + 1, dataCount);
        */
        m_dataCountErrors.incrementAndGet();
        skipIntegration = true;
      }

      if (!skipIntegration) {
        m_processLock.lock();
        m_integ_angle += delta_angle;
        m_y_integ_angle += y_delta_angle;
        m_accelX = accelX;
        m_accelY = accelY;
        m_accelZ = accelZ;
        m_y_gyro_raw = rawGyroY;
        m_processLock.unlock();
      }
      m_dataCount = dataCount;
    }
    previous_timestamp = timestamp;
  }

  /**
   * @param compAngle
   * @param accAngle
   * @return
   */
  private double formatFastConverge(double compAngle, double accAngle) {
    if (compAngle > accAngle + Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    } else if (accAngle > compAngle + Math.PI) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /**
   * @param compAngle
   * @return
   */
  private double formatRange0to2PI(double compAngle) {
    while (compAngle >= 2 * Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    while (compAngle < 0.0) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /**
   * @param accelAngle
   * @param accelZ
   * @return
   */
  private double formatAccelRange(double accelAngle, double accelZ) {
    if (accelZ < 0.0) {
      accelAngle = Math.PI - accelAngle;
    } else if (accelZ > 0.0 && accelAngle < 0.0) {
      accelAngle = 2.0 * Math.PI + accelAngle;
    }
    return accelAngle;
  }

  /** {@inheritDoc} */
  public double getAngle() {
    m_processLock.lock();
    double result = -m_integ_angle;
    m_processLock.unlock();
    return result;
  }

  public double getYAngle() {
    m_processLock.lock();
    double result = m_y_integ_angle;
    m_processLock.unlock();
    return result;
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Units.degreesToRadians(-getAngle()));
  }

  public Rotation2d getYRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getYAngle()));
  }

  public double getAccelerationX() {
    m_processLock.lock();
    double result = m_accelX;
    m_processLock.unlock();
    return result;
  }

  public double getAccelerationY() {
    m_processLock.lock();
    double result = m_accelY;
    m_processLock.unlock();
    return result;
  }

  public double getRawYGyro() {
    m_processLock.lock();
    double result = m_y_gyro_raw;
    m_processLock.unlock();
    return result;
  }

  public double getAccelerationZ() {
    m_processLock.lock();
    double result = m_accelZ;
    m_processLock.unlock();
    return result;
  }

  public Vector<N3> getAcceleration() {
    m_processLock.lock();
    Vector<N3> result = VecBuilder.fill(m_accelX, m_accelY, m_accelZ);
    m_processLock.unlock();
    return result;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("Yaw Degrees CW Pos", () -> getAngle(), null);
    builder.addDoubleProperty("Accel X", () -> getAccelerationX(), null);
    builder.addDoubleProperty("Accel Y", () -> getAccelerationY(), null);
    builder.addDoubleProperty("Accel Z", () -> getAccelerationZ(), null);
    builder.addDoubleProperty("Yaw Radians", () -> getRotation2d().getRadians(), null);
    builder.addDoubleProperty("Y Radians", () -> getYRotation2d().getRadians(), null);
    builder.addDoubleProperty("Y Raw Gyro", () -> getRawYGyro(), null);
    builder.addDoubleProperty("Data Errors", () -> m_dataCountErrors.get(), null);
    builder.addDoubleProperty("Other Errors", () -> m_otherErrorCount.get(), null);
    builder.addIntegerProperty("Read Register Errors", () -> m_readRegisterErrors.get(), null);
    builder.addIntegerProperty("Write Register Errors", () -> m_writeRegisterErrors.get(), null);
  }

  public double getRate() {
    Logger.tag("ADIS16470").error("getRate() not implemented");
    return 0;
  }

  public void setAngle(double angleDegreesCCWPositive) {
    m_processLock.lock();
    m_integ_angle = angleDegreesCCWPositive;
    m_processLock.unlock();
  }
}
