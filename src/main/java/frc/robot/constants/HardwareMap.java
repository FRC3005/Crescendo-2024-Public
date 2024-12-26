package frc.robot.constants;

public final class HardwareMap {
  public final class Intake {
    public static final int kCanIdLeader = 12;
    public static final int kFeederCanId = 18;
    public static final int kIntakeBeamBrakeDigitalPort = 0;
    public static final int kIntakeBeamBrakePowerPort = 1;
    public static final int kFeederBeamBrakeDigitalPort = 4;
    public static final int kFeederBeamBrakePowerPort = 3;
  }

  public final class Launcher {
    public static final int kCanIdTL = 17;
    public static final int kCanIdTR = 14;
    public static final int kCanIdBL = 16;
    public static final int kCanIdBR = 15;
  }

  public final class Arm {
    public static final int kCanId = 10;
    public static final int kThroughBoreDigitalPort = 2;
  }

  public final class Diverter {
    public static final int kCanId = 25;
  }

  public final class Climber {
    public static final int kCanId = 30;
    public static final int kServoPort = 0;
  }

  public final class Drivetrain {
    public static final int kFrontLeftDriveCanId = 3;
    public static final int kRearLeftDriveCanId = 7;
    public static final int kFrontRightDriveCanId = 1;
    public static final int kRearRightDriveCanId = 5;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 6;
  }

  public static final int kBlinkinPwmChannel = 1;
  public static final int kStatusLedPwmChannel = 2;
}
