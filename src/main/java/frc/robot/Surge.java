// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.testmode.TestMode;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.ADIS16470.ADIS16470CalibrationTime;
import frc.robot.constants.AimLookup;
import frc.robot.constants.AutoShootParameters;
import frc.robot.constants.HardwareMap;
import frc.robot.constants.OIConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.indicators.StatusLedStrip8;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diverter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathPlannerCommands;
import frc.robot.subsystems.drive.SwerveCommands;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.robotstate.RobotState;
import org.tinylog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Surge implements RobotContainer {
  private final ADIS16470 m_gyro =
      new ADIS16470(
          RobotConstants.kCompetitionMode
              ? ADIS16470CalibrationTime._8s
              : ADIS16470CalibrationTime._4s);

  // Subsystems
  private final StatusLedStrip8 m_statusLeds =
      new StatusLedStrip8(HardwareMap.kStatusLedPwmChannel);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);
  private final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();
  private final Launcher m_launcher = new Launcher();
  private final Diverter m_diverter = new Diverter();
  private final Climber m_climber = new Climber();
  private final RobotState m_robotState = RobotState.getInstance(m_drive, m_gyro);
  private final Sequencer m_sequencer =
      new Sequencer(m_drive, m_robotState, m_intake, m_arm, m_launcher, m_diverter);

  // State
  private Timer m_armResetTimer = new Timer();
  private boolean m_hasVisitedAuton = false;

  // Controller
  CommandXboxController m_driveController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // Auton
  private final SendableChooser<Command> m_autoChooser;

  // Other things
  private final String kGyroCalSwitch = "Calibrate Gyro (4s)";

  // {
  //   new SwerveTuningForward(m_drive);
  //   SysId.quasistaticForward(m_drive);
  //   SysId.quasistaticReverse(m_drive);
  //   SysId.dynamicForward(m_drive);
  //   SysId.dynamicReverse(m_drive);
  //   SysId.subsystem(m_arm);
  //   SysId.subsystem(m_launcher);
  // }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Surge() {
    AimLookup.initialize();
    Game.registerSmartDashboardOnInit();

    // ALL Spark Maxes should have already been initialized by this point
    SparkFlex.burnFlashInSync();

    RobotController.setBrownoutVoltage(6.3);

    // NOTE: Must call this before creating any auton paths
    m_sequencer.registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    TelemetryRunner.getDefault().bind(m_drive);
    TelemetryRunner.getDefault().bind(m_intake);
    TelemetryRunner.getDefault().bind(m_arm);
    TelemetryRunner.getDefault().bind(m_launcher);
    TelemetryRunner.getDefault().bind(m_robotState);
    TelemetryRunner.getDefault().bind(m_sequencer);
    TelemetryRunner.getDefault().bind(m_climber);
    TelemetryRunner.getDefault().bindSendable(new SendableJVM());
    AutoShootParameters.bindTelemetry(TelemetryRunner.getDefault());

    TestMode.register(m_intake);
    TestMode.register(m_arm);
    TestMode.register(m_launcher);

    // May need to be smarter, but this accounts for blue fixed origin
    // This will only impact practice sessions
    if (Robot.isRedAlliance()) {
      m_gyro.setAngle(180.0);
    }

    m_intake.setRumbleFunction(
        (val) -> m_driveController.getHID().setRumble(RumbleType.kBothRumble, val ? 0.5 : 0.0));

    PathPlannerCommands.createAutoBuilder(m_drive, m_robotState);
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Add any other auton commands after creation of auto builder
    // m_autoChooser.addOption("Shooter Cal", SysId.subsystem(m_launcher));
    // m_autoChooser.addOption("Wheel Diameter Char", m_drive.wheelDiameterCharacterization());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Add names to spark maxes in URCL
    // URCL.start(SparkFlex.getNameMap());

    SmartDashboard.putBoolean(kGyroCalSwitch, false);
  }

  private void configureButtonBindings() {
    /*************************************************
     * Driver controls
     *************************************************/

    m_drive.setDefaultCommand(SwerveCommands.xboxDrive(m_drive, m_driveController));

    m_driveController
        .rightStick()
        .onFalse(
            Commands.runOnce(
                () ->
                    m_drive.setHeading(
                        Robot.isRedAlliance() ? Rotation2d.fromDegrees(180.0) : new Rotation2d())));

    m_driveController.a().onFalse(m_intake.IntakeCommand());
    m_driveController
        .a()
        .and(() -> m_intake.isStoppedOrOutaking())
        .whileTrue(m_intake.IntakeContinuousCommand());
    m_driveController.b().onFalse(m_intake.StopCommand());
    m_driveController.y().onFalse(m_arm.setLowestPositionCommand());
    m_driveController.x().onTrue(Commands.runOnce(() -> m_arm.resetEncoder()));
    m_driveController.start().onFalse(m_intake.OutakeCommand());
    m_driveController.back().onFalse(m_launcher.stopCommand());
    m_driveController.povUp().onTrue(m_climber.extendCommand());
    m_driveController.povDown().whileTrue(m_climber.retractCommand());
    m_driveController.povDown().onFalse(m_climber.lockServoCommand());
    m_driveController
        .leftStick()
        .onTrue(m_sequencer.prepareLowPass())
        .onFalse(m_sequencer.shootAndStow());
    m_driveController
        .leftBumper()
        .and(() -> !m_sequencer.inTestMode())
        .onTrue(m_sequencer.prepareSubwoofer())
        .onFalse(m_sequencer.shootAndStow());
    m_driveController
        .leftBumper()
        .and(() -> m_sequencer.inTestMode())
        .onTrue(m_sequencer.testModeRampUp())
        .onFalse(m_sequencer.testModeShoot());
    m_driveController
        .rightBumper()
        .onTrue(m_sequencer.prepareAmp())
        .onFalse(m_sequencer.shootAmp());
    m_driveController
        .leftTrigger(0.3)
        .and(m_robotState::onGoalSideOfField)
        .whileTrue(m_sequencer.targettingDrive(m_driveController));
    m_driveController
        .leftTrigger(0.3)
        .and(() -> !m_robotState.onGoalSideOfField())
        .whileTrue(m_sequencer.targetShuttleDrive(m_driveController));
    m_driveController.leftTrigger(0.3).onFalse(m_sequencer.stow());
    m_driveController
        .rightTrigger(0.3)
        .and(() -> m_driveController.getLeftTriggerAxis() > 0.3)
        .onTrue(m_sequencer.shoot());
    m_driveController.leftTrigger(0.3).and(m_sequencer::readyToShoot).onTrue(m_sequencer.shoot());

    /*************************************************
     * Combined controls
     *************************************************/

    /*************************************************
     * Operator controls
     *************************************************/

  }

  public void logPeriodic() {
    if (SmartDashboard.getBoolean(kGyroCalSwitch, false)) {
      Logger.tag("Gyro").warn("Running gyro cal!");
      m_gyro.calibrate();
      SmartDashboard.putBoolean(kGyroCalSwitch, false);
    }
  }

  /**
   * Create mappings for use in test mode. This function runs once when this object is created.
   *
   * @return
   */
  private void configureTestModeBindings() {}

  /** This function is called each time testmode is started */
  public void testModeInit() {}

  /**
   * This function is called each loop of testmode. The scheduler is disabled in test mode, so this
   * function should run any necessary loops directly.
   */
  public void testModePeriodic() {
    // m_drive.testPeriodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auton = m_autoChooser.getSelected();

    org.tinylog.Logger.tag("Auton").info("Auton selected: {}", auton.getName());

    return auton;
  }

  /** Run a function at the start of auton. */
  public void autonInit() {
    m_gyro.calibrate();
    m_drive.stop();
    m_intake.stop();
    m_hasVisitedAuton = true;
  }

  public void teleopInit() {
    m_intake.stop();
    // m_drive.setDriveCurrentStrategy(DriveCurrent.kNormal);
  }

  public void disabledInit() {
    m_armResetTimer.restart();
  }

  /**
   * TODO: This is a holdover from amp, due to the way the profile works. The profile is time based
   * and is not based on feedback of where the arm currently is.
   */
  private final double kArmResetFrequency = 2.0;

  public void disabledPeriodic() {
    if (m_armResetTimer.hasElapsed(kArmResetFrequency) && !m_hasVisitedAuton) {
      Logger.tag("DisablePeriodic").info("Resetting Encoder");
      m_arm.resetEncoder();
      m_armResetTimer.restart();

      // Just do this a bunch to make sure this is applied for auton/teleop
      // if (!m_hasVisitedAuton) {
      //   m_drive.setDriveCurrentStrategy(DriveCurrent.kHigh);
      // } else {
      //   m_drive.setDriveCurrentStrategy(DriveCurrent.kNormal);
      // }
    }

    Command autonCommand = m_autoChooser.getSelected();
    String autonName = autonCommand == null ? "" : autonCommand.getName();
    SmartDashboard.putString("Selected Auton", autonName);
    SmartDashboard.putBoolean("HasVistedAuton", m_hasVisitedAuton);
  }
}
