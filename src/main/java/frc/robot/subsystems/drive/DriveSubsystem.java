package frc.robot.subsystems.drive;

import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveModule;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.util.Faults;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.Robot;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.HardwareMap;

public class DriveSubsystem extends SwerveDrive {

  static final SwerveModule frontLeft =
      Robot.isSimulation()
          ? new SimSwerveModule()
          : new REVSwerveModule(
              HardwareMap.Drivetrain.kFrontLeftDriveCanId,
              HardwareMap.Drivetrain.kFrontLeftTurningCanId,
              Math.PI / 2.0);

  static final SwerveModule frontRight =
      Robot.isSimulation()
          ? new SimSwerveModule()
          : new REVSwerveModule(
              HardwareMap.Drivetrain.kFrontRightDriveCanId,
              HardwareMap.Drivetrain.kFrontRightTurningCanId,
              0);

  static final SwerveModule rearLeft =
      Robot.isSimulation()
          ? new SimSwerveModule()
          : new REVSwerveModule(
              HardwareMap.Drivetrain.kRearLeftDriveCanId,
              HardwareMap.Drivetrain.kRearLeftTurningCanId,
              Math.PI);

  static final SwerveModule rearRight =
      Robot.isSimulation()
          ? new SimSwerveModule()
          : new REVSwerveModule(
              HardwareMap.Drivetrain.kRearRightDriveCanId,
              HardwareMap.Drivetrain.kRearRightTurningCanId,
              3.0 * Math.PI / 2.0);

  public DriveSubsystem(ADIS16470 gyro) {
    super(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight,
        DrivetrainConstants.kDriveKinematics,
        gyro,
        DrivetrainConstants.kMaxDriveSpeed,
        DrivetrainConstants.kMaxAngularVelocity);
    // Logger.tag("Swerve Drive").warn("Reset calibration time back to longer for comp");
    Faults.subsystem("Drive").markInitialized();
  }

  public enum DriveCurrent {
    kHigh(DrivetrainConstants.kDriveMotorCurrentLimitAuton),
    kNormal(DrivetrainConstants.kDriveMotorCurrentLimit);

    public final int value;

    private DriveCurrent(int current) {
      value = current;
    }
  }

  public void setDriveCurrentStrategy(DriveCurrent current) {
    frontLeft.setDriveCurrent(current.value);
    frontRight.setDriveCurrent(current.value);
    rearLeft.setDriveCurrent(current.value);
    rearRight.setDriveCurrent(current.value);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.bind(builder);
    builder.bindSendableChild("X Controller", DrivetrainConstants.kXController);
    builder.bindSendableChild("Y Controller", DrivetrainConstants.kYController);
    builder.bindSendableChild("Theta Controller", DrivetrainConstants.kThetaController);
  }
}
