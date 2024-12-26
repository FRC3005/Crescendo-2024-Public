package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.RobotName;
import frc.robot.subsystems.robotstate.Camera;
import org.photonvision.EstimatedRobotPose;

public final class VisionConstants {
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public static Matrix<N3, N1> fixedCameraStdDevCalculation(
      EstimatedRobotPose robotPose, double closestTargetMeters) {
    double xyStdDev = 0.05 * Math.pow(closestTargetMeters, 2.0) / robotPose.targetsUsed.size();
    double thetaStdDev = 0.7 * Math.pow(closestTargetMeters, 2.0) / robotPose.targetsUsed.size();

    // TODO: Try this: if tags uses is greater than 1, use theta
    int tagCount = robotPose.targetsUsed.size();

    // if (RobotState.isAutonomous()) {
    if (tagCount <= 1) {
      thetaStdDev = 1000.0;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public static boolean fixedCameraDistanceAcceptance(
      EstimatedRobotPose robotPose, double closestTargetMeters) {

    int tagCount = robotPose.targetsUsed.size();
    double tagDistance = closestTargetMeters;

    switch (tagCount) {
      case 0:
        return false;
      case 1:
        return tagDistance < 3.0 && robotPose.targetsUsed.get(0).getPoseAmbiguity() < 0.2;
      case 2:
      default:
        return tagDistance < 6.0;
    }
  }

  public static Matrix<N3, N1> zoomCameraStdDevCalculation(
      EstimatedRobotPose robotPose, double closestTargetMeters) {
    double xyStdDev = 0.02 * Math.pow(closestTargetMeters, 2.0) / robotPose.targetsUsed.size();
    double thetaStdDev = 0.08 * Math.pow(closestTargetMeters, 2.0) / robotPose.targetsUsed.size();

    // TODO: Try this: if tags uses is greater than 1, use theta
    int tagCount = robotPose.targetsUsed.size();

    // if (RobotState.isAutonomous()) {
    if (tagCount <= 1) {
      thetaStdDev = 1000.0;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public static boolean zoomCameraDistanceAcceptance(
      EstimatedRobotPose robotPose, double closestTargetMeters) {

    int tagCount = robotPose.targetsUsed.size();
    double tagDistance = closestTargetMeters;

    switch (tagCount) {
      case 0:
        return false;
      case 1:
        return tagDistance < 4.0 && robotPose.targetsUsed.get(0).getPoseAmbiguity() < 0.2;
      default:
        return tagDistance < 8.0;
    }
  }

  public static final String kCamName1 = RobotName.select("baymax", "eva");
  public static final String kCamName2 = RobotName.select("sonny", "noname2");
  public static final Camera kFixedCamera =
      new Camera(
          kCamName1,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-7.295),
                  Units.inchesToMeters(11.0),
                  Units.inchesToMeters(12.274)),
              new Rotation3d(0.0, Units.degreesToRadians(-25.0), Units.degreesToRadians(180.0))),
          VisionConstants::fixedCameraStdDevCalculation,
          VisionConstants::fixedCameraDistanceAcceptance);

  public static final Camera kFrontCamera =
      new Camera(
          kCamName2,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-7.395),
                  Units.inchesToMeters(8.5),
                  Units.inchesToMeters(12.274)),
              new Rotation3d(0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(180.0))),
          VisionConstants::fixedCameraStdDevCalculation,
          VisionConstants::fixedCameraDistanceAcceptance);
}
