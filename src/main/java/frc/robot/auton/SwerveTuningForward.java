package frc.robot.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.lib.auton.AutonCommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveCommands;

public class SwerveTuningForward extends AutonCommandBase {
  public SwerveTuningForward(DriveSubsystem drive) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory("SwerveTuningForward");
    // spotless:off
    addCommandsWithLog(
        SwerveCommands.resetPoseToChoreoState(drive, trajectory.getInitialState()),
        SwerveCommands.choreoTrajectory(drive, trajectory),
        SwerveCommands.stop(drive)
    );
    // spotless:on
  }
}
