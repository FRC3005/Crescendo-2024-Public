package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkSim1;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.SparkFlex;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.lib.vendor.motorcontroller.SparkMonitor;
import org.junit.jupiter.api.*;

public class SparkMaxUtilTest {
  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  @Test
  public void faultToStringTest() {
    short faults =
        (short)
            ((1 << CANSparkBase.FaultID.kBrownout.value)
                | (1 << CANSparkBase.FaultID.kDRVFault.value)
                | (1 << CANSparkBase.FaultID.kOtherFault.value));

    String result = SparkMaxUtils.faultWordToString(faults).strip();
    Assertions.assertEquals("kBrownout kDRVFault kOtherFault", result);
  }

  @Test
  public void monitorTest() {
    SparkFlex spark = new SparkFlex(59);
    CANSparkSim1 sparkMaxSim = new CANSparkSim1(spark);
    SparkMonitor monitor = new SparkMonitor();
    monitor.add(spark);
    sparkMaxSim.setFaults(CANSparkBase.FaultID.kIWDTReset, CANSparkBase.FaultID.kCANRX);
    monitor.periodic();
  }
}
