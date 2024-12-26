package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkSim1;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.testmode.TestableSubsystem;
import frc.lib.util.Faults;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkFlex.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.constants.HardwareMap;

public class Diverter extends TestableSubsystem implements TelemetryNode {
  // Hardware
  private final SparkFlex m_motor = new SparkFlex(HardwareMap.Diverter.kCanId, getName());

  // Settings
  private final boolean kInvert = false;
  private final double kVoltage = 15.0;

  // Sim
  private final CANSparkSim1 m_simMotor = new CANSparkSim1(m_motor);

  public Diverter() {
    m_motor.initialize(
        (isInit) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(m_motor.setIdleMode(IdleMode.kBrake));
          errors += SparkMaxUtils.check(CANSparkBaseExtensions.setInverted(m_motor, kInvert));
          errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(m_motor));
          m_motor.setFrameStrategy(FrameStrategy.kNoFeedback);
          return errors == 0;
        });
    Faults.subsystem("Diverter").markInitialized();
  }

  public Command startCommand() {
    return Commands.runOnce(() -> m_motor.setVoltage(kVoltage), this).withName("Diverter Active");
  }

  public Command stopCommand() {
    return Commands.runOnce(m_motor::stopMotor, this).withName("Diverter Stop");
  }

  @Override
  public void simulationPeriodic() {
    m_simMotor.iterate(10.0, 0.02);
  }

  @Override
  public void toSafeState() {
    m_motor.set(0.0);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    this.initSendable(builder);
    builder.bindChild("Motor", m_motor);
  }
}
