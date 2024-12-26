package frc.robot.subsystems.launcher;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LauncherSetpointTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    // HAL.shutdown();
  }

  @Test
  public void exampleTest() {

    InterpolatingTreeMap<Double, LauncherSetpoint> m_shooterLookup =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), LauncherSetpoint.interpolator());

    m_shooterLookup.put(0.0, new LauncherSetpoint(1000.0, 100.0));
    m_shooterLookup.put(1.0, new LauncherSetpoint(2000.0, 200.0));

    LauncherSetpoint half = m_shooterLookup.get(0.5);
    Assertions.assertEquals(1500.0, half.leftRpm, kEpsilon);
    Assertions.assertEquals(150.0, half.rightRpm, kEpsilon);

    LauncherSetpoint quarter = m_shooterLookup.get(0.25);
    Assertions.assertEquals(1250.0, quarter.leftRpm, kEpsilon);
    Assertions.assertEquals(125.0, quarter.rightRpm, kEpsilon);

    // Beyond the lookup table just gets the last value
    LauncherSetpoint beyond = m_shooterLookup.get(1.5);
    Assertions.assertEquals(2000.0, beyond.leftRpm, kEpsilon);
    Assertions.assertEquals(200.0, beyond.rightRpm, kEpsilon);

    LauncherSetpoint previous = m_shooterLookup.get(-1.0);
    Assertions.assertEquals(1000.0, previous.leftRpm, kEpsilon);
    Assertions.assertEquals(100.0, previous.rightRpm, kEpsilon);
  }
}
