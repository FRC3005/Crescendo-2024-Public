/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.robotstate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.monitor.IsConnected;
import frc.robot.Robot;
import frc.robot.constants.Field;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.tinylog.Logger;

public class Camera implements IsConnected {
  public static record CameraResult(
      String cameraName,
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      List<PhotonTrackedTarget> targets,
      double closestTagDistanceMeters,
      boolean resultsAccepted) {}

  private PhotonCamera m_photonCamera;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private PhotonCameraSim m_photonCameraSim;
  private Field2d m_debugField;
  private double lastEstTimestamp = 0;
  private final Timer m_visionPoseTimer = new Timer();
  private final Timer m_targetLiveTimer = new Timer();
  private static AprilTagFieldLayout m_fieldLayout = null;
  private static final NetworkTable kNetworkTable =
      NetworkTableInstance.getDefault().getTable("Telemetry/Cameras");
  private final StructArrayPublisher<Pose3d> m_targetPublisher;
  private final BiFunction<EstimatedRobotPose, Double, Matrix<N3, N1>> m_calculateStdDevFcn;
  private final BiFunction<EstimatedRobotPose, Double, Boolean> m_acceptanceFilterFcn;

  private DoubleSupplier m_gyroSupplier = null;
  private final VisionClosestToGryo m_singleTagEstimator;

  private static SimCameraProperties kDefaultSimCamera = new SimCameraProperties();

  static {
    kDefaultSimCamera.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
    kDefaultSimCamera.setCalibError(0.35, 0.10);
    kDefaultSimCamera.setFPS(25);
    kDefaultSimCamera.setAvgLatencyMs(50);
    kDefaultSimCamera.setLatencyStdDevMs(15);
  }

  public Camera(
      String name,
      Transform3d robotToCamera,
      BiFunction<EstimatedRobotPose, Double, Matrix<N3, N1>> calculateStdDevFcn,
      BiFunction<EstimatedRobotPose, Double, Boolean> acceptanceFilter) {
    this(name, robotToCamera, calculateStdDevFcn, acceptanceFilter, kDefaultSimCamera);
  }

  public void setGyroSupplier(DoubleSupplier gyroSupplier) {
    m_gyroSupplier = gyroSupplier;
  }

  public Camera(
      String name,
      Transform3d robotToCamera,
      BiFunction<EstimatedRobotPose, Double, Matrix<N3, N1>> calculateStdDevFcn,
      BiFunction<EstimatedRobotPose, Double, Boolean> acceptanceFilter,
      SimCameraProperties simProperties) {
    m_calculateStdDevFcn = calculateStdDevFcn;
    m_acceptanceFilterFcn = acceptanceFilter;
    m_targetLiveTimer.restart();
    m_photonCamera = new PhotonCamera(name);

    m_targetPublisher = kNetworkTable.getStructArrayTopic(name, Pose3d.struct).publish();

    if (m_fieldLayout == null) {
      try {
        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
        // field.
        // Load Deploy/competition-field-layout.json first if its there, otherwise load default
        // This allows us to tweak the actual field locations if needed.
        try {
          m_fieldLayout =
              new AprilTagFieldLayout(
                  Filesystem.getDeployDirectory().getAbsolutePath()
                      + "competition-field-layout.json");
          Logger.tag("Camera").info("Using custom field layout");
        } catch (Exception e) {
          m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
          Logger.tag("Camera").info("Using built-in field layout");
        }
      } catch (Exception e) {
        m_fieldLayout = null;
        Logger.tag("Camera").error("Failed to load AprilTagFieldLayout\r\n{}", e.toString());
      }
    }

    m_singleTagEstimator = new VisionClosestToGryo(robotToCamera, m_fieldLayout);

    if (m_fieldLayout == null) {
      m_photonPoseEstimator = null;
    } else {
      // Create pose estimator
      m_photonPoseEstimator =
          new PhotonPoseEstimator(
              m_fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              m_photonCamera,
              robotToCamera);
      // Ideally this should be reference pose fallback, but threading makes it more complex
      // m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
      m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    if (Robot.isSimulation()) {
      Logger.tag(name).info("Creating camera in simulation");

      m_photonCameraSim = new PhotonCameraSim(m_photonCamera, simProperties);
      m_photonCameraSim.enableDrawWireframe(true);
      m_photonCameraSim.enableRawStream(true);
      m_photonCameraSim.enableProcessedStream(true);
      // Add the simulated camera to view the targets on this simulated field.
      // simSystem.addCamera(m_photonCameraSim, Vision.kCameraLocations.get(name));
    }
  }

  public void setDebugField(Field2d debugField) {
    m_debugField = debugField;
  }

  private static boolean isTargetAcceptable(EstimatedRobotPose robotPose, double tagDistance) {
    int tagCount = robotPose.targetsUsed.size();

    if (tagCount <= 0) {
      return false;
    }

    // Stay on the field
    if (robotPose.estimatedPose.getX() < -Field.kBorderMargin
        || robotPose.estimatedPose.getX() > Field.kLength + Field.kBorderMargin
        || robotPose.estimatedPose.getY() < -Field.kBorderMargin
        || robotPose.estimatedPose.getY() > Field.kWidth + Field.kBorderMargin) {
      return false;
    }

    // Check that the pose height makes sense
    // TODO Check This, then remove it
    SmartDashboard.putNumber("Camera/Height", robotPose.estimatedPose.getZ());
    if (Math.abs(robotPose.estimatedPose.getZ()) > 0.75) {
      return false;
    }

    return true;
  }

  public String getCameraName() {
    return m_photonCamera.getName();
  }

  public PhotonCameraSim getSimCamera() {
    return m_photonCameraSim;
  }

  public Transform3d getRobotToCameraTransform() {
    return m_photonPoseEstimator.getRobotToCameraTransform();
  }

  /**
   * Return the direct latest result from the PV API. Use this when not using pose estimation
   * functions.
   *
   * @return latest result
   */
  public PhotonPipelineResult getLatestPipelineResult() {
    return m_photonCamera.getLatestResult();
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (m_photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    PhotonPipelineResult pipelineResult = m_photonCamera.getLatestResult();
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    Optional<EstimatedRobotPose> estimate;
    if (pipelineResult.getTargets().size() == 1 && m_gyroSupplier != null) {
      estimate = m_singleTagEstimator.estimate(pipelineResult, m_gyroSupplier.getAsDouble());
    } else {
      estimate = m_photonPoseEstimator.update(pipelineResult);
    }
    double latestTimestamp = pipelineResult.getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation() && m_debugField != null) {
      estimate.ifPresentOrElse(
          est -> m_debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) m_debugField.getObject("VisionEstimation").setPoses();
          });
    }

    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    return estimate;
  }

  @Override
  public boolean isConnected() {
    return m_photonCamera.isConnected();
  }

  public Optional<CameraResult> getCameraResult(Pose2d referencePose) {
    m_visionPoseTimer.restart();
    Optional<EstimatedRobotPose> res = this.getEstimatedGlobalPose(referencePose);
    SmartDashboard.putNumber("Camera/Get Pose Timer", m_visionPoseTimer.get());

    // Possible optimization, currently disabled. Only use this if multiple tags are seen.
    if (res.isPresent()) {
      EstimatedRobotPose robotPose = res.get();
      Pose2d pose2d = robotPose.estimatedPose.toPose2d();
      // Find closest tag distance (meters)
      double distance = 100;
      Pose3d[] targetPoses = new Pose3d[robotPose.targetsUsed.size()];
      for (int i = 0; i < robotPose.targetsUsed.size(); i++) {
        var target = robotPose.targetsUsed.get(i);
        double t_distance =
            target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        if (t_distance < distance) {
          distance = t_distance;
        }

        // Placeholder for sim
        if (t_distance < Units.feetToMeters(20.0)) {
          targetPoses[i] =
              robotPose
                  .estimatedPose
                  .plus(m_photonPoseEstimator.getRobotToCameraTransform())
                  .plus(target.getBestCameraToTarget());
        } else {
          targetPoses[i] = robotPose.estimatedPose;
        }
      }

      if (isTargetAcceptable(robotPose, distance)) {
        m_targetLiveTimer.restart();
        m_targetPublisher.set(targetPoses);
        CameraResult result =
            new CameraResult(
                m_photonCamera.getName(),
                pose2d,
                robotPose.timestampSeconds,
                m_calculateStdDevFcn.apply(robotPose, distance),
                robotPose.targetsUsed,
                distance,
                m_acceptanceFilterFcn.apply(robotPose, distance));
        return Optional.of(result);
      }
    }

    if (m_targetLiveTimer.hasElapsed(0.25)) {
      m_targetPublisher.set(new Pose3d[0]);
    }
    return Optional.empty();
  }
}
