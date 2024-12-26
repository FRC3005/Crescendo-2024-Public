package frc.robot.subsystems.drive;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.swerve.SwerveAccelerationLimiter;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveDriverInput;
import frc.lib.swerve.SwerveHeadingController;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.util.JoystickUtil;
import frc.robot.Robot;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.OIConstants;
import java.util.function.Supplier;
import org.tinylog.Logger;

// TODO: Move out year specific and make this part of library
public class SwerveCommands {

  public static Command choreoTrajectory(SwerveDrive drive, ChoreoTrajectory trajectory) {
    // TODO: Pass this in from somewhere
    String trajectoryName = "";

    ChoreoControlFunction controller =
        Choreo.choreoSwerveController(
            DrivetrainConstants.kXController,
            DrivetrainConstants.kYController,
            DrivetrainConstants.kThetaController);

    ProtobufPublisher<Pose2d> currentPosePublisher =
        NetworkTableInstance.getDefault()
            .getProtobufTopic("/choreo/currentPose", Pose2d.proto)
            .publish();
    ProtobufPublisher<Pose2d> refPosePublisher =
        NetworkTableInstance.getDefault()
            .getProtobufTopic("/choreo/refPose", Pose2d.proto)
            .publish();
    ProtobufPublisher<ChassisSpeeds> chassisSpeedsRefPublisher =
        NetworkTableInstance.getDefault()
            .getProtobufTopic("/choreo/refChassisSpeeds", ChassisSpeeds.proto)
            .publish();
    StructArrayPublisher<Pose2d> trajectoryPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/choreo/trajectory", Pose2d.struct)
            .publish();

    return Commands.sequence(
        // Update Logging
        Commands.runOnce(
            () -> {
              trajectoryPublisher.set(trajectory.getPoses());
              Logger.tag("Choreo").trace("Starting trajectory {}", trajectoryName);
            }),

        // Run command
        Choreo.choreoSwerveCommand(
            trajectory,
            drive::getPose,
            (pose, state) -> {
              currentPosePublisher.set(pose);
              refPosePublisher.set(state.getPose());
              chassisSpeedsRefPublisher.set(state.getChassisSpeeds());
              return controller.apply(pose, state);
            },
            (ChassisSpeeds speeds) -> drive.drive(speeds),
            Robot::isRedAlliance,
            drive),

        // Update logging
        Commands.runOnce(
            () -> Logger.tag("Choreo").trace("Finished trajectory {}", trajectoryName)));
  }

  public static Command stop(SwerveDrive drive) {
    return Commands.runOnce(drive::stop, drive);
  }

  public static Command resetPoseToChoreoState(SwerveDrive drive, ChoreoTrajectoryState state) {
    return Commands.runOnce(
        () -> {
          var correctedState = Robot.isRedAlliance() ? state.flipped() : state;
          drive.setPose(correctedState.getPose());
        });
  }

  private static ChassisSpeeds xboxDriveTransform(
      SwerveDrive drive, CommandXboxController gamepad) {
    SwerveDriverInput inputs = new SwerveDriverInput(drive);
    var axis =
        JoystickUtil.circleDeadband(
            -gamepad.getLeftX(), -gamepad.getLeftY(), OIConstants.kDriveDeadband);
    double leftX = JoystickUtil.squareAxis(axis.xAxis);
    double leftY = JoystickUtil.squareAxis(axis.yAxis);
    var rightX = JoystickUtil.squareAxis(-gamepad.getRightX(), OIConstants.kDriveDeadband);

    // When using blue fixed 0 coordinate system, must invert all
    if (Robot.isRedAlliance()) {
      leftX = -leftX;
      leftY = -leftY;
    }

    return inputs.drive(leftY, leftX, rightX, true);
  }

  public static Command xboxDrive(SwerveDrive drive, CommandXboxController gamepad) {
    return Commands.run(() -> drive.drive(xboxDriveTransform(drive, gamepad)), drive)
        .withName("Default Drive");
  }

  // TODO: Move this to a better spot!!
  public static PIDController m_pidController = new PIDController(10.0, 0, 0.25);
  private static double m_accelerationLimit = 10.0;
  private static double m_velocityScalar = 0.4;
  private static SwerveAccelerationLimiter m_accelerationLimiter = null;

  public static Command xboxLockHeadingDrive(
      SwerveDrive drive,
      CommandXboxController gamepad,
      PIDController pidController,
      double velocityScalar,
      Supplier<Rotation2d> supplier) {
    SwerveHeadingController controller =
        new SwerveHeadingController(drive, pidController, velocityScalar, supplier);
    SmartDashboard.putData(controller);

    return Commands.run(
            () -> {

              // TODO: Remove tunability once this is figured out
              if (m_accelerationLimiter == null) {
                m_accelerationLimiter = new SwerveAccelerationLimiter(m_accelerationLimit);
                SmartDashboard.putNumber("Acceleration Limit on Lock", m_accelerationLimit);
              }

              double accelLimit =
                  SmartDashboard.getNumber("Acceleration Limit on Lock", m_accelerationLimit);
              if (accelLimit != m_accelerationLimit) {
                m_accelerationLimit = accelLimit;
                m_accelerationLimiter.setAccelerationLimit(m_accelerationLimit);
                Logger.tag("SwerveCommands")
                    .info("Setting acceleration limit to {}", m_accelerationLimit);
              }

              var chassisSpeeds =
                  controller.transform(
                      m_accelerationLimiter.transform(xboxDriveTransform(drive, gamepad)));
              drive.drive(chassisSpeeds);
            },
            drive)
        .withName("Xbox Lock Heading Drive");
  }

  public static Command lockHeadingDrive(
      SwerveDrive drive, ChassisSpeeds chassisSpeeds, Supplier<Rotation2d> supplier) {
    SwerveHeadingController controller =
        new SwerveHeadingController(drive, m_pidController, supplier);
    SmartDashboard.putData(controller);
    return Commands.run(() -> drive.drive(controller.transform(chassisSpeeds)), drive)
        .withName("Lock Heading Drive");
  }

  public static Command xboxLockHeadingDrive(
      SwerveDrive drive, CommandXboxController gamepad, Supplier<Rotation2d> supplier) {
    return xboxLockHeadingDrive(drive, gamepad, m_pidController, m_velocityScalar, supplier);
  }

  public static Command xboxLockHeadingDrive(
      SwerveDrive drive,
      CommandXboxController gamepad,
      double velocityScalar,
      Supplier<Rotation2d> supplier) {
    return xboxLockHeadingDrive(drive, gamepad, m_pidController, velocityScalar, supplier);
  }

  public static void bind(TelemetryRunner runner) {
    runner.bindSendable(m_pidController);
  }
}
