package frc.lib;

import com.revrobotics.CANSparkSim1;
import edu.wpi.first.hal.HAL;
import frc.lib.controller.Controller;
import frc.lib.controller.PIDGains;
import frc.lib.vendor.motorcontroller.SparkFlex;
import org.junit.jupiter.api.*;

public class SparkMaxControllerTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  @Test
  public void controllerWrapTest() {
    SparkFlex spark = new SparkFlex(1);
    Controller controller = spark.positionController(new PIDGains());
    CANSparkSim1 sim = new CANSparkSim1(spark);

    controller.enableContinuousInput(-Math.PI, Math.PI);

    controller.setReference(Math.PI, (Math.PI / 2) * 4 * Math.PI);

    // Assertions.assertEquals("Unwrapped value", 5 * Math.PI, sim.getSetpoint(), kEpsilon);
  }
}
