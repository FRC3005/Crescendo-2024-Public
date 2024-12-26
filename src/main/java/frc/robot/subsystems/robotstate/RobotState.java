package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveDrive;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.util.Faults;
import frc.lib.util.Faults.Fault;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.Game;
import frc.robot.Robot;
import frc.robot.constants.AimLookup;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.indicators.StatusLedStrip8;
import frc.robot.subsystems.robotstate.Camera.CameraResult;
import frc.robot.thirdparty.LimelightHelpers;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotState extends SubsystemBase implements TelemetryNode {

  private static RobotState m_instance = null;
  private final SwerveDrive m_swerve;

  private final ADIS16470 m_gyro;

  // Use specific numbers well off the field to make it obvious this is on purpose
  private static final Pose2d kInvalidPose = new Pose2d(-1337, 3005, new Rotation2d());

  private final Field2d m_field = new Field2d();
  private boolean m_disableVision = false;
  private Fault m_fault = Faults.subsystem("RobotState");
  private Pose2d m_futurePose = new Pose2d();

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private boolean m_hasTargetInSight = false;

  private TargetResult m_targetResult = new TargetResult();
  private TargetResult m_futureTargetResult = new TargetResult();
  private TargetResult m_shuttleTargetResult = new TargetResult();

  private final Timer m_lastTagInView = new Timer();

  private HashMap<String, Double> m_tagPoseTimestamp = new HashMap<>();

  private VisionSystemSim m_simVison = new VisionSystemSim("main");
  private final Camera m_fixedCamera = VisionConstants.kFixedCamera;
  private final Camera m_frontCamera = VisionConstants.kFrontCamera;
  private final String kLimelightName = "limelight-tags";

  public static RobotState getInstance(SwerveDrive swerve, ADIS16470 gyro) {
    if (m_instance == null) {
      m_instance = new RobotState(swerve, gyro);
    }
    return m_instance;
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  private RobotState(SwerveDrive swerve, ADIS16470 gyro) {
    Rotation2d initialRotation =
        Robot.isRedAlliance() ? Rotation2d.fromDegrees(180.0) : new Rotation2d();
    m_swerve = swerve;
    m_gyro = gyro;
    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            DrivetrainConstants.kEnableCarpetCorrection
                ? DrivetrainConstants.kDriveCarpetKinematics
                : DrivetrainConstants.kDriveKinematics,
            initialRotation,
            m_swerve.getModulePositions(),
            new Pose2d(new Translation2d(), initialRotation),
            VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(40)));
    m_simVison.addAprilTags(VisionConstants.kTagLayout);
    m_lastTagInView.restart();

    DoubleSupplier gyroSupplier = () -> m_gyro.getRotation2d().getRadians();

    // Use gyro for single tag
    // m_fixedCamera.setGyroSupplier(gyroSupplier);
    // m_frontCamera.setGyroSupplier(gyroSupplier);

    if (Robot.isSimulation()) {
      m_simVison.addCamera(m_fixedCamera.getSimCamera(), m_fixedCamera.getRobotToCameraTransform());
      m_simVison.addCamera(m_frontCamera.getSimCamera(), m_frontCamera.getRobotToCameraTransform());
    }
  }

  /**
   * Get the pose of the robot using fused vision and wheel odometry measurements.
   *
   * @return pose of the robot based on vision and odometry measurements.
   */
  public Pose2d getPoseEstimate() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the pose of the robot where it would if shooting from that point, the moment the note
   * passes through the entry to the goal.
   *
   * @return future pose of the robot when the gamepiece enters the goal
   */
  public Pose2d getPoseEstimateForShot() {
    return m_futurePose;
  }

  private Pose2d getPoseEstimateForShotInternal() {
    Pose2d currentPose = getPoseEstimate();
    double currentDistanceToTarget = getTargetResult().distance();
    Translation2d targetTranslation = Game.getSpeakerShotTranslation();

    /*
     * When in motion, you can't use current distance to target to calculate the lookup, you must
     * use the future distance to target. But looking up that distance requires looking up using
     * the current distance. Simple approach here is to iterate a few times, using the distance
     * from each subsuquent lookup.
     *
     * TODO: Some logging is included to get rough idea of how accurate it is, and how many iterations
     * it took. Do some analysis on this.
     *
     * TODO: Return ending accuracy to allow a decision based on it.
     */
    double distanceToTarget = currentDistanceToTarget;
    Pose2d futurePose =
        currentPose.exp(
            m_swerve.getPredictedMotion(
                AimLookup.flightTime(distanceToTarget)
                    + IntakeConstants.kFeedTimeToShooterSeconds));
    int i = 0;
    for (; i < 5; i++) {
      double newDistToTarget = futurePose.getTranslation().getDistance(targetTranslation);

      if (Math.abs(distanceToTarget - newDistToTarget) < 0.001) {
        distanceToTarget = newDistToTarget;
        break;
      }
      distanceToTarget = newDistToTarget;

      futurePose =
          currentPose.exp(
              m_swerve.getPredictedMotion(
                  AimLookup.flightTime(distanceToTarget)
                      + IntakeConstants.kFeedTimeToShooterSeconds));
    }

    double newDistToTarget = futurePose.getTranslation().getDistance(targetTranslation);
    SmartDashboard.putNumber(
        "Shot Est Pose Distance Error", Math.abs(distanceToTarget - newDistToTarget));
    SmartDashboard.putNumber("Shot Est Iterations", i);

    return futurePose;
  }

  /**
   * Reset the pose estimation to a fixed pose.
   *
   * @param resetPose pose to set the estimator to.
   */
  public void resetPoseEstimate(Pose2d resetPose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_swerve.getModulePositions(), resetPose);
  }

  public void disableVision() {
    m_disableVision = true;
  }

  public void enableVision() {
    m_disableVision = false;
  }

  public boolean isVisionEnabled() {
    return !m_disableVision;
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return m_simVison.getDebugField();
  }

  public boolean isAutonomous() {
    return edu.wpi.first.wpilibj.RobotState.isAutonomous();
  }

  public boolean isTeleop() {
    return edu.wpi.first.wpilibj.RobotState.isTeleop();
  }

  private void processLimelight() {
    LimelightHelpers.SetRobotOrientation(
        kLimelightName, m_gyro.getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
    if (mt2.tagCount > 0) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      m_field.getObject(kLimelightName).setPose(mt2.pose);
    }
  }

  private void getCameraResultAndUpdate(Camera camera) {
    Optional<CameraResult> result = camera.getCameraResult(m_poseEstimator.getEstimatedPosition());
    if (result.isPresent() && isVisionEnabled()) {
      var cameraResult = result.get();

      m_lastTagInView.restart();

      if (cameraResult.resultsAccepted()) {
        m_poseEstimator.addVisionMeasurement(
            cameraResult.visionRobotPoseMeters(),
            cameraResult.timestampSeconds(),
            cameraResult.visionMeasurementStdDevs());
        m_field.getObject(cameraResult.cameraName()).setPose(cameraResult.visionRobotPoseMeters());
        m_tagPoseTimestamp.put(cameraResult.cameraName(), cameraResult.timestampSeconds());
      }

      // Check for target in view even if we reject the frame
      checkForTarget(cameraResult.targets());
    }
  }

  @Override
  public void periodic() {
    // Workaround: Issue where module CAN data is corrupt
    var modulePositions = m_swerve.getModulePositions();
    boolean skip = false;
    for (var position : modulePositions) {
      if (position.distanceMeters == 0.0) {
        skip = true;
        break;
      }
    }
    SmartDashboard.putBoolean("skip", skip);

    if (skip) {
      m_fault.error("Invalid odom data detected, skipping");
    } else {
      m_fault.clear();
      // TODO: Get timestamps for swerve odometry or otherwise correct for it
      m_poseEstimator.update(m_gyro.getRotation2d(), m_swerve.getModulePositions());
    }

    if (!m_fixedCamera.isConnected()) {
      m_fault.error("Fixed camera not connected!");
    }

    if (!m_frontCamera.isConnected()) {
      m_fault.error("Front camera not connected!");
    }

    getCameraResultAndUpdate(m_fixedCamera);

    // if (isTeleop()) {
    getCameraResultAndUpdate(m_frontCamera);
    // }
    // processLimelight();

    // Invalidate old field poses for telemetry
    for (var cameraPoseTimestamp : m_tagPoseTimestamp.entrySet()) {
      double timeNow = Timer.getFPGATimestamp();

      if (timeNow - cameraPoseTimestamp.getValue() > 1.0) {
        m_field.getObject(cameraPoseTimestamp.getKey()).setPose(kInvalidPose);
      }
    }

    // Calcuate current target info
    m_targetResult =
        getTargetResultInternal(
            getPoseEstimate(), Game.getSpeakerShotTranslation(), m_hasTargetInSight);

    m_futurePose = getPoseEstimateForShotInternal();

    // Calculate future target location
    m_futureTargetResult =
        getTargetResultInternal(m_futurePose, Game.getSpeakerShotTranslation(), true);

    // Calculate shuttle target location
    m_shuttleTargetResult =
        getTargetResultInternal(getPoseEstimate(), Game.getLongDistanceShuttleLocation(), true);

    if (!m_lastTagInView.hasElapsed(0.5)) {
      StatusLedStrip8.setHasTagInView(true);
    } else {
      StatusLedStrip8.setHasTagInView(false);
    }

    Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();
    m_field.getObject("Future Pose").setPose(m_futurePose);
    m_field.setRobotPose(estimatedPose);
    DrivetrainConstants.kDriveCarpetKinematics.setRobotPose(estimatedPose);
  }

  public void resetSwerveOdometryToEstimate() {
    m_swerve.setPose(m_poseEstimator.getEstimatedPosition());
  }

  private void checkForTarget(List<PhotonTrackedTarget> targets) {
    int tagId = Game.getSpeakerCenterTagId();

    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == tagId) {
        m_hasTargetInSight = true;
        return;
      }
    }
    m_hasTargetInSight = false;
  }

  private Transform3d m_targetDirect = new Transform3d();

  /**
   * Get the rotation to the target based on the measured angle to the target itself. Fall back to
   * estimated pose if it does not have the specific target in sight.
   *
   * <p>This is based on the team and target ID after any fixed offsets.
   *
   * @return robot relative rotation to the target. For example if the robot is turned clockwise 20
   *     degrees from the target (-20 degrees in field terms) this will return +20 degrees. NULL if
   *     not seen.
   */
  private Transform3d getRobotToTargetDirectInternal(double targetTimeout) {
    PhotonPipelineResult result = m_fixedCamera.getLatestPipelineResult();
    double resultTimestamp = result.getTimestampSeconds();

    // Invalid timestamp can be set
    if (resultTimestamp < 0) {
      return null;
    }

    double timeDelta = Timer.getFPGATimestamp() - resultTimestamp;

    if (timeDelta > targetTimeout) {
      return null;
    }

    int tagId = Game.getSpeakerCenterTagId();
    Transform3d cameraToTarget = null;

    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == tagId) {
        SmartDashboard.putNumber("YAW", target.getYaw());
        // Use the simple 'non-calibrated'
        // if (Robot.isSimulation()) {
        //   rotation = Rotation2d.fromDegrees(target.getYaw());
        // } else {
        //   rotation = Rotation2d.fromDegrees(-target.getYaw());
        // }

        // 'calibrated' result
        cameraToTarget =
            target.getBestCameraToTarget().plus(m_fixedCamera.getRobotToCameraTransform());
        break;
      }
    }

    if (cameraToTarget == null) {
      return null;
    } else {
      return cameraToTarget;
    }
  }

  private TargetResult getTargetResultInternal(
      Pose2d pose, Translation2d targetTranslation, boolean targetInSight) {
    Transform2d robotToTarget =
        new Transform2d(pose, new Pose2d(targetTranslation, new Rotation2d()));
    return new TargetResult(
        pose.getTranslation().getDistance(targetTranslation),
        robotToTarget.getTranslation().getAngle().plus(Rotation2d.fromDegrees(180.0)),
        targetInSight);
  }

  public TargetResult getTargetResult() {
    return m_targetResult;
  }

  public TargetResult getShuttleTargetResult() {
    return m_shuttleTargetResult;
  }

  public TargetResult getFutureTargetResult() {
    return m_futureTargetResult;
  }

  public boolean hasSightlineToTarget() {
    return m_hasTargetInSight;
  }

  private final double kRedXThreshold = 9.5;
  private final double kBlueXThreshold = 7.0;
  private final double kThresholdMargin = 1.25;
  private double m_fieldSideThreshold = kRedXThreshold;

  public boolean onGoalSideOfField() {
    double xPos = m_poseEstimator.getEstimatedPosition().getX();
    if (Robot.isRedAlliance()) {
      if (xPos > m_fieldSideThreshold) {
        m_fieldSideThreshold = kRedXThreshold - kThresholdMargin;
        return true;
      } else {
        m_fieldSideThreshold = kRedXThreshold;
        return false;
      }
    } else {
      if (xPos < m_fieldSideThreshold) {
        m_fieldSideThreshold = kBlueXThreshold + kThresholdMargin;
        return true;
      } else {
        m_fieldSideThreshold = kBlueXThreshold;
        return false;
      }
    }
  }

  private void bindTargetResult(
      TelemetryBuilder builder, String name, Supplier<TargetResult> result) {
    builder.addDoubleProperty(name + " distance (m)", () -> result.get().distance(), null);
    builder.addDoubleProperty(
        name + " rotation (deg)", () -> result.get().rotation().getDegrees(), null);
    builder.addBooleanProperty(name + " in view", () -> result.get().inView(), null);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindSendableChild("Field", m_field);
    builder.addDoubleProperty("battery voltage", () -> RobotController.getBatteryVoltage(), null);
    builder.addBooleanProperty("brownout", () -> RobotController.isBrownedOut(), null);
    if (!RobotConstants.kReducedTelemetryMode) {
      builder.addBooleanProperty(
          "Reset Odometry to Vision", () -> false, (val) -> resetSwerveOdometryToEstimate());
      builder.addDoubleProperty("Distance to goal", () -> getTargetResult().distance(), null);
      builder.addBooleanProperty("On goal side", this::onGoalSideOfField, null);
      builder.addDoubleProperty("Goal side threshold", () -> m_fieldSideThreshold, null);
      builder.addBooleanProperty("Fixed Camera Connected", () -> m_fixedCamera.isConnected(), null);
      builder.addBooleanProperty("Front Camera Connected", () -> m_frontCamera.isConnected(), null);
      bindTargetResult(builder, "target", () -> m_targetResult);
      bindTargetResult(builder, "future target", () -> m_futureTargetResult);
      bindTargetResult(builder, "shuttle", () -> m_shuttleTargetResult);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_simVison.update(m_swerve.getPose());
    this.testModePeriodic();
    m_fixedCamera.setDebugField(m_simVison.getDebugField());
  }

  public void testModePeriodic() {
    periodic();
  }
}
