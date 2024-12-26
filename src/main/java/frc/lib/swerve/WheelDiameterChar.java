package frc.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.tinylog.Logger;

public class WheelDiameterChar extends Command {
  enum States {
    IDLE,
    RUNNING
  }

  private record TestPoint(
      double gyroRadians, double[] wheelDistanceRadians, double timestampSeconds) {
    public TestPoint(double gyroRadians, double[] wheelDistanceRadians) {
      this(gyroRadians, wheelDistanceRadians, Timer.getFPGATimestamp());
    }
  }

  private static final NetworkTable kNetworkTable =
      NetworkTableInstance.getDefault().getTable("tests/swerveWheelDiameterTest");

  // 0.25 RPS (4 seconds per rotation)
  private static final double kRotationSpeedRadiansPerSecond = Math.PI / 2.0;

  private final StringPublisher m_testStatePublisher =
      kNetworkTable.getStringTopic("state").publish();
  private final DoubleArrayPublisher m_startPointWheelRadiansPublisher =
      kNetworkTable.getDoubleArrayTopic("starting wheel radians").publish();
  private final DoublePublisher m_startPointGyroPublisher =
      kNetworkTable.getDoubleTopic("starting gyro").publish();
  private final DoubleArrayPublisher m_endPointWheelRadiansPublisher =
      kNetworkTable.getDoubleArrayTopic("ending wheel radians").publish();
  private final DoublePublisher m_endPointGyroPublisher =
      kNetworkTable.getDoubleTopic("ending gyro").publish();
  private final DoublePublisher m_wheelDiameterPublisher =
      kNetworkTable.getDoubleTopic("wheel diameter").publish();

  private final Timer m_testTimer = new Timer();
  private final DoubleSupplier m_gyroSupplier;
  private final Supplier<double[]> m_wheelDistanceSupplier;
  private TestPoint m_startPoint;
  private TestPoint m_endPoint;
  private final double m_testDuration;
  private final double m_driveBaseRadius;

  private final SwerveDrive m_drive;

  /**
   * @param drive
   * @param gyroAngleRadiansSupplier radians, but doesn't actually matter CCW positive vs CW
   *     positive
   */
  public WheelDiameterChar(
      SwerveDrive drive,
      Supplier<double[]> wheelDistanceRadiansSupplier,
      DoubleSupplier gyroAngleRadiansSupplier,
      double driveBaseRadiusMeters,
      double testTimeSeconds) {
    m_drive = drive;
    m_gyroSupplier = gyroAngleRadiansSupplier;
    m_wheelDistanceSupplier = wheelDistanceRadiansSupplier;
    m_testStatePublisher.set(States.IDLE.name());
    m_testDuration = testTimeSeconds;
    m_driveBaseRadius = driveBaseRadiusMeters;

    addRequirements(m_drive);
  }

  public TestPoint captureSample() {
    return new TestPoint(m_gyroSupplier.getAsDouble(), m_wheelDistanceSupplier.get());
  }

  @Override
  public void initialize() {
    m_testStatePublisher.set(States.RUNNING.name());
    m_startPoint = null; // captureSample();

    // m_startPointWheelRadiansPublisher.accept(m_startPoint.wheelDistanceRadians());
    // m_startPointGyroPublisher.accept(m_startPoint.gyroRadians());

    m_testTimer.restart();
  }

  @Override
  public void execute() {
    if (m_testTimer.hasElapsed(1.0) && m_startPoint == null) {
      m_startPoint = captureSample();
      m_startPointWheelRadiansPublisher.accept(m_startPoint.wheelDistanceRadians());
      m_startPointGyroPublisher.accept(m_startPoint.gyroRadians());
    }
    if (m_testTimer.hasElapsed(m_testDuration)) {
      m_drive.stop();
    } else {
      m_drive.drive(new ChassisSpeeds(0.0, 0.0, kRotationSpeedRadiansPerSecond));
    }
  }

  private double calculateRadius(TestPoint start, TestPoint end) {
    int startingNumModules = start.wheelDistanceRadians().length;
    int endingNumModules = end.wheelDistanceRadians().length;
    int numModules = Math.min(startingNumModules, endingNumModules);

    if (endingNumModules != startingNumModules) {
      Logger.tag("WheelDiameterChar")
          .warn(
              "Number of swerver modules changed during the test! Starting: {}, Ending: {}. Continuing with {}, but the value may be off!",
              startingNumModules,
              endingNumModules,
              numModules);
    }

    double gyroDelta = Math.abs(end.gyroRadians() - start.gyroRadians());

    if (gyroDelta < Units.degreesToRadians(5.0)) {
      Logger.tag("WheelDiameterChar")
          .warn("Gyro delta {} is very small, results may be impacted!", gyroDelta);
    }

    double meanDiameter = 0.0;
    for (int i = 0; i < numModules; i++) {
      double wheelRadiansDelta =
          Math.abs(end.wheelDistanceRadians()[i] - start.wheelDistanceRadians()[i]);

      if (wheelRadiansDelta < Units.inchesToMeters(3.0)) {
        Logger.tag("WheelDiameterChar")
            .warn(
                "Wheel delta {} (m) for module {} is very small, results may be impacted!",
                wheelRadiansDelta,
                i);
      }

      double radius = gyroDelta * m_driveBaseRadius / wheelRadiansDelta;
      Logger.tag("WheelDiameterChar").info("Wheel diameter module {} is {} (m)", i, radius * 2);
      meanDiameter += 2 * radius;
    }
    Logger.tag("WheelDiameterChar").info("Wheel diameter mean {} (m)", meanDiameter / numModules);

    return meanDiameter / numModules;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_testTimer.stop();
    m_endPoint = captureSample();
    m_testStatePublisher.set(States.IDLE.name());

    m_endPointWheelRadiansPublisher.accept(m_endPoint.wheelDistanceRadians());
    m_endPointGyroPublisher.accept(m_endPoint.gyroRadians());
    m_wheelDiameterPublisher.accept(
        Units.metersToInches(calculateRadius(m_startPoint, m_endPoint)));
  }

  @Override
  public boolean isFinished() {
    return m_testTimer.hasElapsed(m_testDuration)
        && m_drive.getChassisSpeeds().omegaRadiansPerSecond < 0.0001;
  }
}
