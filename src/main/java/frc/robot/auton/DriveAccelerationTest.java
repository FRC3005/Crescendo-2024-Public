package frc.robot.auton;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auton.AutonCommandBase;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.tinylog.Logger;

public class DriveAccelerationTest extends AutonCommandBase {
  // Test settings
  private static final double kSprintDistancesMetersDefault = 8.0;
  private static final double kDecelerationRate = 1.0;
  private static final double kDriveSpeedDefault = 0.6;
  private double m_angleGain = 0.5;

  // Test state
  private double m_driveSpeed = kDriveSpeedDefault;

  private final SlewRateLimiter m_limiter = new SlewRateLimiter(kDecelerationRate);
  private double m_testStartTimestamp = 0.0;
  private double m_interruptTimestamp = 0.0;

  private static final NetworkTable kNetworkTable =
      NetworkTableInstance.getDefault().getTable("tests/driveAccelerationTest");
  private static final DoublePublisher m_startTimePublisher =
      kNetworkTable.getDoubleTopic("results/testStartTime").publish();
  private static final DoublePublisher m_endTimePublisher =
      kNetworkTable.getDoubleTopic("results/testEndTime").publish();
  private static final DoublePublisher m_sprintTimePublisher =
      kNetworkTable.getDoubleTopic("results/sprintTime").publish();
  private static final StringPublisher m_testStatePublisher =
      kNetworkTable.getStringTopic("testState").publish();
  private static final DoubleSubscriber m_sprintDistanceSubscriber =
      kNetworkTable.getDoubleTopic("sprintDistanceMeters").subscribe(kSprintDistancesMetersDefault);
  private static final DoubleSubscriber m_driveSpeedSubscriber =
      kNetworkTable.getDoubleTopic("sprintDistanceMeters").subscribe(kDriveSpeedDefault);

  public DriveAccelerationTest(DriveSubsystem drive, DigitalInput lineSensor, ADIS16470 gyro) {
    SynchronousInterrupt interrupt = new SynchronousInterrupt(lineSensor);
    m_testStatePublisher.accept("Idle");
    addRequirements(drive);

    addCommandsWithLog(
        // Reset the heading to 0.0
        Commands.runOnce(
                () -> {
                  m_driveSpeed = m_driveSpeedSubscriber.get();
                  drive.setHeading(new Rotation2d());
                  drive.setPose(new Pose2d());
                  Logger.tag("DriveAccelerationTest")
                      .info("Starting test, drive speed: {}", m_driveSpeed);
                  m_testStatePublisher.accept("Running");
                  m_testStartTimestamp = Timer.getFPGATimestamp();
                })
            .withName("Acceleration Test Init"),

        // Drive full speed no ramping until we either hit the switch, or reach max distance
        // Keep the bot straight (0 degrees) using the gyro
        Commands.run(
                () -> {
                  drive.drive(
                      m_driveSpeed, 0.0, gyro.getRotation2d().getRadians() * -m_angleGain, false);

                  // drive.driveVoltage(m_driveSpeed, new Rotation2d());

                  var timestamp = interrupt.getRisingTimestamp();
                  if (timestamp > m_testStartTimestamp) {
                    m_interruptTimestamp = timestamp;
                  }
                })
            .withName("Acceleration Test Run")
            .until(
                () ->
                    (drive.getPose().getX() > m_sprintDistanceSubscriber.get())
                        || m_interruptTimestamp > m_testStartTimestamp)
            .withName("DriveAcceleration Run"),

        // Log the data
        Commands.runOnce(
                () -> {
                  if (m_interruptTimestamp > m_testStartTimestamp) {
                    Logger.tag("DriveAccelerationTest")
                        .info(
                            "Starting ramp down. Sprint time: {} - Start time {} - End time {}",
                            m_testStartTimestamp - m_interruptTimestamp,
                            m_testStartTimestamp,
                            m_interruptTimestamp);
                  } else {
                    Logger.tag("DriveAccelerationTest")
                        .warn("Starting ramp down due to distance expired, did not see tag");
                  }
                  m_testStatePublisher.accept("Ramp Down");
                  m_startTimePublisher.accept(m_testStartTimestamp);
                  m_endTimePublisher.accept(m_interruptTimestamp);
                  m_sprintTimePublisher.accept(m_interruptTimestamp - m_testStartTimestamp);
                })
            .withName("Acceleration Test Log"),

        // Next, ramp throttle to zero at some rate
        Commands.runOnce(() -> m_limiter.reset(m_driveSpeed))
            .withName("Acceleration Test Reset Limiter"),
        Commands.run(() -> drive.drive(m_limiter.calculate(0.0), 0.0, 0.0, false))
            .withName("Acceleration Test Ramp Down")
            .finallyDo(
                () -> {
                  drive.stop();
                  Logger.tag("DriveAccelerationTest").info("Test concluded.");
                  m_testStatePublisher.accept("Idle");
                })
            .withName("Acceleration Test End"));
  }
}
