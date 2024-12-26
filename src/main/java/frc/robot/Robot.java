// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.testmode.TestMode;
import frc.lib.util.Version;
import frc.robot.constants.RobotConstants;
import java.util.Optional;
import org.littletonrobotics.urcl.URCL;
import org.tinylog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static final double kLoggerPeriod = 0.02;
  private final StringPublisher m_statePublisher =
      NetworkTableInstance.getDefault().getTable("StateData").getStringTopic("Mode").publish();

  // Use for uptime counter, always off by default, then on on enable
  PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_statePublisher.set("robotInit");
    // Start logging before anything else
    DataLogManager.start();
    var log = DataLogManager.getLog();
    log.addSchema(Pose2d.proto);
    log.addSchema(ChassisSpeeds.proto);
    log.addSchema(Pose2d.struct);
    log.addSchema(Pose3d.struct);

    URCL.start();
    LogConfig.config();

    DriverStation.silenceJoystickConnectionWarning(!RobotConstants.kCompetitionMode);
    DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new Surge(); // new SwerveBot();
    addPeriodic(m_robotContainer::logPeriodic, kLoggerPeriod);

    // StaticHTTPServer.startServer(null);

    Logger.tag("RobotMain")
        .info(
            "Robot code starting in {} mode. Version: {}, Hash: {}",
            RobotConstants.kCompetitionMode ? "competition" : "practice",
            Version.get());

    SmartDashboard.putString("Version", Version.get());

    pd.setSwitchableChannel(false);

    // Disable all LiveWindow functionality
    LiveWindow.setEnabledListener(null);
    LiveWindow.setDisabledListener(null);
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();

    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (!RobotConstants.kCompetitionMode) {
      // Flush network table:ws when testing to get higher resolution graphs. This will bump it up
      // from
      // 10 Hz to 50 Hz. This can hurt CPU usage so be careful when using it.
      NetworkTableInstance.getDefault().flush();
    }
    TelemetryRunner.getDefault().update();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_statePublisher.set("disabledInit");
    pd.setSwitchableChannel(false);
    m_robotContainer.disabledInit();
    TestMode.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  /** This autonomous runs the autonomous command selected by your {@link Amp} class. */
  @Override
  public void autonomousInit() {
    double startTime = Timer.getFPGATimestamp();
    m_statePublisher.set("autonInit");
    m_robotContainer.autonInit();
    pd.setSwitchableChannel(true);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Try to limit init time to a max of 50ms
    double initTime = Timer.getFPGATimestamp() - startTime;
    if (initTime < 0.05) {
      double sleepTime = MathUtil.clamp(0.05 - initTime, 0.0, 0.05);
      Logger.tag("Match Info")
          .info("Fast init was {} seconds, waiting {} seconds", initTime, sleepTime);
      Timer.delay(sleepTime);
    }

    Logger.tag("Match Info")
        .info("Robot.java Auton initialize took {} seconds.", Timer.getFPGATimestamp() - startTime);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_statePublisher.set("teleopInit");
    pd.setSwitchableChannel(true);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    m_statePublisher.set("testInit");
    pd.setSwitchableChannel(true);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testModeInit();

    TestMode.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.testModePeriodic();
    TestMode.periodic();
  }

  public static boolean isRedAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  @Override
  public void autonomousExit() {
    if (DriverStation.isFMSAttached()) {
      Logger.tag("Match Info")
          .info(
              "Event: {}, Match: {} {}, Replay: {}, Robot: {}-{}, Game Message: {}, Time: {}",
              DriverStation.getEventName(),
              DriverStation.getMatchType(),
              DriverStation.getMatchNumber(),
              DriverStation.getReplayNumber(),
              DriverStation.getAlliance(),
              DriverStation.getLocation(),
              DriverStation.getGameSpecificMessage(),
              DriverStation.getMatchTime());
    } else {
      Logger.tag("Match Info")
          .info(
              "Not an official match: {}, Robot: {}-{}",
              DriverStation.getMatchNumber(),
              DriverStation.getAlliance(),
              DriverStation.getLocation());
    }
  }
}
