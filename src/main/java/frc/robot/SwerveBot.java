// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.auton.AutonChooser;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.ADIS16470.ADIS16470CalibrationTime;
import frc.robot.auton.DriveAccelerationTest;
import frc.robot.auton.SwerveTuningForward;
import frc.robot.auton.SysId;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveCommands;
import frc.robot.subsystems.robotstate.RobotState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveBot implements RobotContainer {
  private final ADIS16470 m_gyro =
      new ADIS16470(
          RobotConstants.kCompetitionMode
              ? ADIS16470CalibrationTime._8s
              : ADIS16470CalibrationTime._1s);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);
  private final RobotState m_robotState = RobotState.getInstance(m_drive, m_gyro);

  CommandXboxController m_driveController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private String m_selectedAutonName = "";

  private final DigitalInput m_lineSensor = new DigitalInput(9);

  {
    if (!RobotConstants.kCompetitionMode) {
      new DriveAccelerationTest(m_drive, m_lineSensor, m_gyro);
      new SwerveTuningForward(m_drive);
      SysId.quasistaticForward(m_drive);
      SysId.quasistaticReverse(m_drive);
      SysId.dynamicForward(m_drive);
      SysId.dynamicReverse(m_drive);
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveBot() {

    // ALL Spark Maxes should have already been initialized by this point
    SparkFlex.burnFlashInSync();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    TelemetryRunner.getDefault().bind(m_drive);
    TelemetryRunner.getDefault().bind(m_robotState);
    TelemetryRunner.getDefault().bindSendable(new SendableJVM());
    TelemetryRunner.getDefault().bindSendable(m_lineSensor);
    SwerveCommands.bind(TelemetryRunner.getDefault());

    if (Robot.isSimulation()) {
      m_drive.setPose(new Pose2d(3.0, 5.0, new Rotation2d()));
    }
  }

  private void configureButtonBindings() {
    /*************************************************
     * Driver controls
     *************************************************/

    var fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    m_drive.setDefaultCommand(SwerveCommands.xboxDrive(m_drive, m_driveController));

    m_driveController
        .rightStick()
        .onFalse(Commands.runOnce(() -> m_drive.zeroHeading()).withName("Zero Drive Heading"));

    m_driveController
        .a()
        .onTrue(
            SwerveCommands.xboxLockHeadingDrive(
                    m_drive,
                    m_driveController,
                    () -> {
                      return m_drive.getHeading().plus(m_robotState.getTargetResult().rotation());
                    })
                .until(() -> Math.abs(m_driveController.getHID().getRightX()) > 0.15));

    // Probably refactor the below (more likely just remove as not needed). Useful for testing angle
    // lock controller.
    m_driveController
        .povDown()
        .onFalse(
            SwerveCommands.xboxLockHeadingDrive(
                    m_drive, m_driveController, () -> Rotation2d.fromDegrees(180.0))
                .until(() -> Math.abs(m_driveController.getHID().getRightX()) > 0.15));
    m_driveController
        .povUp()
        .onFalse(
            SwerveCommands.xboxLockHeadingDrive(
                    m_drive, m_driveController, () -> Rotation2d.fromDegrees(0.0))
                .until(() -> Math.abs(m_driveController.getHID().getRightX()) > 0.15));
    m_driveController
        .povRight()
        .onFalse(
            SwerveCommands.xboxLockHeadingDrive(
                    m_drive, m_driveController, () -> Rotation2d.fromDegrees(-90.0))
                .until(() -> Math.abs(m_driveController.getHID().getRightX()) > 0.15));
    m_driveController
        .povLeft()
        .onFalse(
            SwerveCommands.xboxLockHeadingDrive(
                    m_drive, m_driveController, () -> Rotation2d.fromDegrees(90.0))
                .until(() -> Math.abs(m_driveController.getHID().getRightX()) > 0.15));

    /*************************************************
     * Combined controls
     *************************************************/

    /*************************************************
     * Operator controls
     *************************************************/

  }

  public void logPeriodic() {}

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
    m_drive.testPeriodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auton = AutonChooser.getAuton();

    org.tinylog.Logger.tag("Auton").info("Auton selected: {}", auton.getName());

    m_selectedAutonName = auton.getName();

    return auton;
  }

  /** Run a function at the start of auton. */
  public void autonInit() {
    m_gyro.calibrate();
    m_drive.stop();
  }

  public void teleopInit() {}

  public void disabledInit() {}

  public void disabledPeriodic() {}
}
