package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.robotstate.RobotState;
import java.util.Optional;
import org.tinylog.Logger;

public class PathPlannerCommands {
  public static Command followPath(
      DriveSubsystem drive, RobotState robotState, PathPlannerPath path) {
    return new FollowPathHolonomic(
        path,
        robotState::getPoseEstimate,
        drive::getChassisSpeeds,
        drive::drive,
        DrivetrainConstants.kPathPlannerConfig,
        Robot::isRedAlliance,
        drive);
  }

  public static void createAutoBuilder(DriveSubsystem drive, RobotState robotState) {
    AutoBuilder.configureHolonomic(
        robotState::getPoseEstimate,
        (val) -> {
          // Set gyro first!
          Logger.tag("Auton").trace("Setting swerve pose and heading {}", val);
          drive.setHeading(val.getRotation());
          drive.setPose(val);
          robotState.resetPoseEstimate(val);
        },
        drive::getChassisSpeeds,
        drive::drive,
        DrivetrainConstants.kPathPlannerConfig,
        Robot::isRedAlliance,
        drive);
    // PPHolonomicDriveController.setRotationTargetOverride(PathPlannerCommands::getRotationTargetOverride);
  }

  private static boolean rotationOverrideEnable = false;

  public static void enableRotationOverride() {
    rotationOverrideEnable = true;
  }

  public static void disableRotationOverride() {
    rotationOverrideEnable = false;
  }

  public static Optional<Rotation2d> getRotationTargetOverride() {
    if (!rotationOverrideEnable) {
      return Optional.empty();
    }
    return Optional.empty();
  }
}
