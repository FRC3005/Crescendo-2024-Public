package frc.robot.subsystems.robotstate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionClosestToGryo {
  private final Transform3d robotToCamera;
  private final AprilTagFieldLayout fieldTags;

  public VisionClosestToGryo(Transform3d robotToCamera, AprilTagFieldLayout fieldTags) {
    this.robotToCamera = robotToCamera;
    this.fieldTags = fieldTags;
  }

  /**
   * Return the estimated position of the robot using the target with the lowest delta in the vector
   * magnitude between it and the reference pose.
   *
   * @param result pipeline result
   * @param referencePose reference pose to check vector magnitude difference against.
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this
   *     estimation.
   */
  public Optional<EstimatedRobotPose> estimate(
      PhotonPipelineResult result, double gyroAngleRadians) {
    double smallestPoseDelta = 10e9;
    EstimatedRobotPose lowestDeltaPose = null;

    for (PhotonTrackedTarget target : result.targets) {
      int targetFiducialId = target.getFiducialId();

      // Don't report errors for non-fiducial targets. This could also be resolved by
      // adding -1 to
      // the initial HashSet.
      if (targetFiducialId == -1) continue;

      Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

      if (targetPosition.isEmpty()) {
        // TODO: This reports (once) for invalid april tags
        // reportFiducialPoseError(targetFiducialId);
        continue;
      }

      Pose3d altTransformPosition =
          targetPosition
              .get()
              .transformBy(target.getAlternateCameraToTarget().inverse())
              .transformBy(robotToCamera.inverse());
      Pose3d bestTransformPosition =
          targetPosition
              .get()
              .transformBy(target.getBestCameraToTarget().inverse())
              .transformBy(robotToCamera.inverse());

      double altDifference = Math.abs(gyroAngleRadians - altTransformPosition.getRotation().getZ());
      double bestDifference =
          Math.abs(gyroAngleRadians - bestTransformPosition.getRotation().getZ());

      SmartDashboard.putNumber("alt Diff", altDifference);
      SmartDashboard.putNumber("Best Diff", bestDifference);
      SmartDashboard.putNumber("Gyro rads", gyroAngleRadians);
      SmartDashboard.putNumber("alt rads", altTransformPosition.getRotation().getZ());
      SmartDashboard.putNumber("best rads", bestTransformPosition.getRotation().getZ());

      if (altDifference < smallestPoseDelta) {
        smallestPoseDelta = altDifference;
        lowestDeltaPose =
            new EstimatedRobotPose(
                altTransformPosition,
                result.getTimestampSeconds(),
                result.getTargets(),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      }
      if (bestDifference < smallestPoseDelta) {
        smallestPoseDelta = bestDifference;
        lowestDeltaPose =
            new EstimatedRobotPose(
                bestTransformPosition,
                result.getTimestampSeconds(),
                result.getTargets(),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      }
    }
    if (lowestDeltaPose != null) {
      SmartDashboard.putNumber("Selected!!", lowestDeltaPose.estimatedPose.getRotation().getZ());
    }
    return Optional.ofNullable(lowestDeltaPose);
  }
}
