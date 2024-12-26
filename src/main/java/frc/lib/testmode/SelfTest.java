package frc.lib.testmode;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class SelfTest {
  private final Command m_command;
  private final BooleanSupplier m_passingCriteria;

  public SelfTest(Command command, BooleanSupplier passingCriteria) {
    m_command = command;
    m_passingCriteria = passingCriteria;
  }

  public boolean isRunning() {
    return !m_command.isFinished();
  }

  public void run() {
    m_command.schedule();
  }

  public boolean didPass() {
    return m_passingCriteria.getAsBoolean();
  }
}
