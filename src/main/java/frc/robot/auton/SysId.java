package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.auton.AutonCommandBase;
import frc.lib.testmode.TestableSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SysId extends AutonCommandBase {
  private SysId(Command command, String name) {
    super(name);
    addCommandsWithLog(command);
  }

  public static SysId quasistaticForward(DriveSubsystem drive) {
    return new SysId(drive.sysIdDrive().quasistatic(Direction.kForward), "DriveQuasistaticForward");
  }

  public static SysId quasistaticReverse(DriveSubsystem drive) {
    return new SysId(drive.sysIdDrive().quasistatic(Direction.kReverse), "DriveQuasistaticReverse");
  }

  public static SysId dynamicForward(DriveSubsystem drive) {
    return new SysId(drive.sysIdDrive().dynamic(Direction.kForward), "DriveDynamicForward");
  }

  public static SysId dynamicReverse(DriveSubsystem drive) {
    return new SysId(drive.sysIdDrive().dynamic(Direction.kReverse), "DriveDynamicReverse");
  }

  public static SysId subsystem(TestableSubsystem subsystem) {
    return new SysId(subsystem.sysId(), subsystem.getName());
  }

  public static SysId rotationSysid(DriveSubsystem drive) {
    return new SysId(
        drive
            .sysIdRotation(8.0)
            .quasistatic(Direction.kForward)
            .andThen(Commands.waitSeconds(2))
            .andThen(
                drive
                    .sysIdRotation(8.0)
                    .quasistatic(Direction.kReverse)
                    .andThen(Commands.waitSeconds(2))
                    .andThen(drive.sysIdRotation(4.0).dynamic(Direction.kForward))
                    .andThen(Commands.waitSeconds(2))
                    .andThen(drive.sysIdRotation(4.0).dynamic(Direction.kReverse))),
        "rotationSysidAllInOne");
  }
}
