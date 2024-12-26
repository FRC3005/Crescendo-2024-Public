package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.robot.constants.AimLookup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.AutoShootParameters;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveCommands;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherSetpoint;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.TargetResult;

public class Sequencer implements TelemetryNode {
  private final Intake m_intake;
  private final Arm m_arm;
  private final Launcher m_launcher;
  private final Diverter m_diverter;
  private final DriveSubsystem m_drive;
  private final RobotState m_robotState;

  private final String kSmartDashboardKey = "Disable Auto Shoot";
  private final String kShootAndMoveSDKey = "Disable Shoot and Move";

  public Sequencer(
      DriveSubsystem drive,
      RobotState state,
      Intake intake,
      Arm arm,
      Launcher launcher,
      Diverter diverter) {
    m_intake = intake;
    m_arm = arm;
    m_launcher = launcher;
    m_diverter = diverter;
    m_drive = drive;
    m_robotState = state;
    SmartDashboard.putBoolean(kSmartDashboardKey, false);
    SmartDashboard.putBoolean(kShootAndMoveSDKey, false);
  }

  public boolean allowShootAndMove() {
    return !SmartDashboard.getBoolean(kShootAndMoveSDKey, false);
  }

  private TargetResult getTargetResult() {
    return allowShootAndMove()
        ? m_robotState.getFutureTargetResult()
        : m_robotState.getTargetResult();
  }

  public Command shootAmp() {
    return Commands.sequence(
            m_diverter.startCommand(),
            m_intake.FeedCommandNoRequirements(),
            Commands.waitSeconds(1.0),
            m_intake.StopCommandNoRequirements(),
            m_launcher.idleCommand(),
            m_diverter.stopCommand(),
            m_arm.setPositionCommand(Rotation2d.fromDegrees(18.0)))
        .withName("shootAmp");
  }

  public Command shootAndStow() {
    return Commands.sequence(
            m_intake.FeedCommand(),
            Commands.waitSeconds(0.5),
            m_intake.StopCommand(),
            m_launcher.idleCommand(),
            m_arm.setPositionCommand(Rotation2d.fromDegrees(18.0)))
        .withName("shootAndStow");
  }

  public Command stow() {
    return Commands.sequence(
            m_intake.StopCommandNoRequirements(),
            m_launcher.idleCommand(),
            m_arm.setLowestPositionCommand())
        .withName("stow");
  }

  public Command shoot() {
    return shoot(1.0);
  }

  public Command shoot(double shootTime) {
    return Commands.sequence(
            m_intake.FeedCommand(), Commands.waitSeconds(shootTime), m_intake.StopCommand())
        .withName("shoot");
  }

  private final double kSubwooferDistanceMeters = 2.1;

  public Command testModeRampUp() {
    return m_launcher.shootTuningShotCommand();
  }

  public Command testModeShoot() {
    return shoot();
  }

  public Command prepareLowPass() {
    return Commands.parallel(
            m_arm.setPositionCommand(Rotation2d.fromDegrees(21.0)),
            m_launcher.shootFixedRpm(new LauncherSetpoint(LauncherConstants.kLowPassRpm)))
        .withName("prepareLowShot");
  }

  public Command prepareSubwoofer() {
    return Commands.parallel(
            m_arm.setPositionCommand(Rotation2d.fromDegrees(53.0)),
            m_launcher.shootFixedRpm(new LauncherSetpoint(3143.0)))
        .withName("prepareSubwoofer");
  }

  public Command prepareAmp() {
    return Commands.parallel(
            m_arm.setPositionCommand(Rotation2d.fromDegrees(94.0)), m_launcher.ampCommand())
        .withName("prepareAmp");
  }

  public Command targettingDrive(CommandXboxController controller) {
    return Commands.parallel(
            SwerveCommands.xboxLockHeadingDrive(
                m_drive,
                controller,
                () -> {
                  return m_drive.getHeading().plus(getTargetResult().rotation());
                }),
            m_arm.trackGoalCommand(() -> AimLookup.armAngle(getTargetResult().distance())),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(getTargetResult().distance())))
        .withName("Target Goal");
  }

  public Command targetShuttleDrive(CommandXboxController controller) {
    return Commands.parallel(
            SwerveCommands.xboxLockHeadingDrive(
                m_drive,
                controller,
                1.0,
                () -> {
                  return m_drive
                      .getHeading()
                      .plus(m_robotState.getShuttleTargetResult().rotation());
                }),
            m_arm.trackGoalCommand(
                () -> AimLookup.shuttleArmAngle(m_robotState.getShuttleTargetResult().distance())),
            m_launcher.shootTrackCommand(
                () -> AimLookup.shuttleSetpoint(m_robotState.getShuttleTargetResult().distance())))
        .withName("Target Shuttle");
  }

  private double calculateOnTheMoveMaxVelocity() {
    // Distance between max distance and min distance
    double distance = getTargetResult().distance();
    double ratio =
        (distance - AutoShootParameters.kMinShootDistanceMetersOnTheMove)
            / (AutoShootParameters.kMaxShootDistanceMetersOnTheMove
                - AutoShootParameters.kMinShootDistanceMetersOnTheMove);
    ratio = MathUtil.clamp(ratio, 0.0, 1.0);
    return MathUtil.interpolate(
        AutoShootParameters.kMaxTransalationalVelocityMetersPerSecondOnTheMove,
        AutoShootParameters.kMaxTransalationalVelocityMetersPerSecond,
        ratio);
  }

  public boolean readyToShoot() {
    ChassisSpeeds chassisSpeeds = m_drive.getChassisSpeeds();
    double translationalVelocity =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double rotationalVelocity = Math.abs(chassisSpeeds.omegaRadiansPerSecond);

    double maxTranslationalVelocity =
        allowShootAndMove()
            ? calculateOnTheMoveMaxVelocity()
            : AutoShootParameters.kMaxTransalationalVelocityMetersPerSecond;

    double maxRotationalVelocity =
        allowShootAndMove()
            ? AutoShootParameters.kMaxRotationalVelocityRadiansPerSecondOnTheMove
            : AutoShootParameters.kMaxRotationalVelocityRadiansPerSecond;

    boolean enabled = !SmartDashboard.getBoolean(kSmartDashboardKey, false);
    boolean hasTarget = m_robotState.hasSightlineToTarget();
    boolean hasGamepiece = m_intake.intakeHasGamePiece() || m_intake.feederHasGamePiece();
    boolean slowEnough = translationalVelocity <= maxTranslationalVelocity;
    boolean rotSlowEnough = rotationalVelocity <= maxRotationalVelocity;

    boolean closeEnough =
        m_robotState.getTargetResult().distance() <= AutoShootParameters.kMaxShootDistanceMeters;

    boolean armReady = m_arm.atSetpoint(AutoShootParameters.kMaxArmAngleErrorRadians);
    boolean launcherReady = m_launcher.atSetpoint(AutoShootParameters.kRpmTolerance);
    boolean angleOnTarget =
        Math.abs(getTargetResult().rotation().getRadians())
            <= AutoShootParameters.kMaxAngularErrorRadians;

    SmartDashboard.putNumber("AutoShoot/maxVel", maxTranslationalVelocity);
    SmartDashboard.putBoolean("AutoShoot/enabled", enabled);
    SmartDashboard.putBoolean("AutoShoot/hasTarget", hasTarget);
    SmartDashboard.putBoolean("AutoShoot/hasGamepiece", hasGamepiece);
    SmartDashboard.putBoolean("AutoShoot/slowEnough", slowEnough);
    SmartDashboard.putBoolean("AutoShoot/rotSlowEnough", rotSlowEnough);
    SmartDashboard.putBoolean("AutoShoot/closeEnough", closeEnough);
    SmartDashboard.putBoolean("AutoShoot/armReady", armReady);
    SmartDashboard.putBoolean("AutoShoot/launcherReady", launcherReady);
    SmartDashboard.putBoolean("AutoShoot/angleOnTarget", angleOnTarget);

    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      enabled = true;
      hasGamepiece = true;
    }

    return enabled
        && hasTarget
        && hasGamepiece
        && slowEnough
        && rotSlowEnough
        && closeEnough
        && armReady
        && launcherReady
        && angleOnTarget;
  }

  /*******
   * Auton
   */
  public Command shootSubwooferAuton() {
    return Commands.parallel(
            m_arm.setPositionCommand(Rotation2d.fromDegrees(57.0)), m_launcher.shootCloseCommand())
        .andThen(Commands.waitUntil(() -> m_launcher.atSetpoint(750.0)))
        .andThen(shoot(0.25))
        .withName("autonSubwoofer");
  }

  public Command autonShootAtStartNotSubwoofer() {
    return Commands.sequence(
        m_intake.StopCommand(),
        startArmAndLauncherTracking().until(() -> m_launcher.atSetpoint(350.0)),
        shootStoppedThenIdle());
  }

  public Command autonTargetAndShootStopped() {
    return Commands.parallel(
            SwerveCommands.lockHeadingDrive(
                m_drive,
                new ChassisSpeeds(),
                () -> m_drive.getHeading().plus(m_robotState.getTargetResult().rotation())),
            m_arm.trackGoalCommand(
                () -> AimLookup.armAngle(m_robotState.getTargetResult().distance())),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(m_robotState.getTargetResult().distance())))
        .raceWith(Commands.sequence(shoot(0.6)))
        .withName("Auton Target Goal");
  }

  public Command shootStoppedThenIdle() {
    return Commands.sequence(
            Commands.runOnce(m_drive::stop, m_drive), shoot(0.2), m_launcher.idleCommand())
        .withName("Shoot Stopped then idle");
  }

  public Command shootThenIdle() {
    return shoot(0.2).andThen(m_launcher.idleCommand()).withName("Shoot and Idle");
  }

  public Command startArmAndLauncherTrackingFixedOffset(Rotation2d armOffset) {
    return Commands.parallel(
            m_arm.trackGoalCommand(
                () ->
                    AimLookup.armAngle(m_robotState.getTargetResult().distance())
                        + armOffset.getRadians()),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(m_robotState.getTargetResult().distance())))
        .withName("Start Tracking Arm Launcher Arm Offset: " + armOffset);
  }

  public Command startArmAndLauncherTracking() {
    return Commands.parallel(
            m_arm.trackGoalCommand(
                () -> AimLookup.armAngle(m_robotState.getTargetResult().distance())),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(m_robotState.getTargetResult().distance())))
        .withName("Start Tracking Arm Launcher");
  }

  public Command startArmAndLauncherTrackingOnTheMove() {
    return Commands.parallel(
            m_arm.trackGoalCommand(
                () -> AimLookup.armAngle(m_robotState.getFutureTargetResult().distance())),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(m_robotState.getFutureTargetResult().distance())))
        .withName("Start Tracking Arm Launcher On The Move");
  }

  public Command startArmAndLauncherTrackingOnTheMoveOffset(Rotation2d offset) {
    return Commands.parallel(
            m_arm.trackGoalCommand(
                () ->
                    AimLookup.armAngle(m_robotState.getFutureTargetResult().distance())
                        + offset.getRadians()),
            m_launcher.shootTrackCommand(
                () -> AimLookup.launcherSetpoint(m_robotState.getFutureTargetResult().distance())))
        .withName("Start Tracking Arm Launcher On The Move Offset: " + offset);
  }

  public Command outtakeTimed() {
    return Commands.sequence(
            m_intake.OutakeCommand(), Commands.waitSeconds(1.5), m_intake.StopCommand())
        .withName("outtake timed");
  }

  private final double kTwoAutonFarShotsArmOffset = -0.2;
  private final double kFirstShotArmOffset = 0.0;

  public void registerNamedCommands() {
    NamedCommands.registerCommand("shootSubwoofer", shootSubwooferAuton());
    NamedCommands.registerCommand("shoot", shoot(0.2));
    NamedCommands.registerCommand("intake", m_intake.IntakeCommand());
    NamedCommands.registerCommand("outtake", outtakeTimed());
    NamedCommands.registerCommand("targetAndShoot", autonTargetAndShootStopped());
    NamedCommands.registerCommand("shootThenIdle", shootStoppedThenIdle());
    NamedCommands.registerCommand("startTrackingArmLauncher", startArmAndLauncherTracking());
    NamedCommands.registerCommand(
        "startTrackingArmLauncherOffset",
        startArmAndLauncherTrackingFixedOffset(Rotation2d.fromDegrees(kTwoAutonFarShotsArmOffset)));
    NamedCommands.registerCommand(
        "startTrackingArmLauncherOffsetOnTheMove",
        startArmAndLauncherTrackingOnTheMoveOffset(Rotation2d.fromDegrees(kFirstShotArmOffset)));
    NamedCommands.registerCommand(
        "startTrackingArmLauncherOnTheMove", startArmAndLauncherTrackingOnTheMove());
    NamedCommands.registerCommand(
        "lowerArm", m_arm.setPositionCommand(Rotation2d.fromDegrees(20.0)));
    NamedCommands.registerCommand("stow", stow());
    NamedCommands.registerCommand("launcherIdle", m_launcher.idleCommand());
    NamedCommands.registerCommand("autonShootStartNotSubwoofer", autonShootAtStartNotSubwoofer());
    NamedCommands.registerCommand("runFeeder", m_intake.FeedCommand());
    NamedCommands.registerCommand(
        "launcherPlop", m_launcher.shootFixedRpm(new LauncherSetpoint(450.0, 450.0)));
    NamedCommands.registerCommand("intakeFeedthrough", m_intake.FeedThroughCommand());
    NamedCommands.registerCommand("stopIntake", m_intake.StopCommand());
    NamedCommands.registerCommand("armUnderStage", m_arm.enableArmUnderStageCommand());
    NamedCommands.registerCommand("armNotUnderStage", m_arm.disableArmUnderStageCommand());
    NamedCommands.registerCommand(
        "armBloop", m_arm.setPositionCommand(Rotation2d.fromDegrees(22.0)));
    NamedCommands.registerCommand(
        "armLow", m_arm.setPositionCommand(Rotation2d.fromDegrees(ArmConstants.kMinRotation)));
  }

  private boolean m_testmode = false;

  public boolean inTestMode() {
    return m_testmode && !DriverStation.isFMSAttached();
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.addBooleanProperty("Enable Test Mode", this::inTestMode, (val) -> m_testmode = val);
  }
}
