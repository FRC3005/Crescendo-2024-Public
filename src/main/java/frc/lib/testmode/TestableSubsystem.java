package frc.lib.testmode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TestableSubsystem extends SubsystemBase {
  public void toSafeState() {}

  public SelfTest selfTest() {
    return null;
  }

  public Command sysId() {
    return Commands.none();
  }
}
