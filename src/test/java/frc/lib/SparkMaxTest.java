package frc.lib;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.SparkFlex;
import java.util.function.BiFunction;
import org.junit.jupiter.api.*;

public class SparkMaxTest {

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  private static final BiFunction<CANSparkFlex, Boolean, Boolean> initFunction =
      (CANSparkFlex spark, Boolean initializer) -> {
        spark.setIdleMode(IdleMode.kBrake);
        spark.setSmartCurrentLimit(20);

        spark.enableVoltageCompensation(12.12);
        return true;
      };

  @Test
  public void basicDriverTest() {
    SparkFlex spark = new SparkFlex(4, MotorType.kBrushless).withInitializer(initFunction);

    int devId = spark.getDeviceId();
    var tmp = spark.getIdleMode();

    Assertions.assertEquals(4, devId);

    Assertions.assertEquals(IdleMode.kBrake, tmp);

    spark.mutate(
        (innerSpark, init) -> {
          innerSpark.setIdleMode(IdleMode.kCoast);
          return true;
        });

    tmp = spark.getIdleMode();
    Assertions.assertEquals(IdleMode.kCoast, tmp);
  }
}
