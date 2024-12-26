package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.*;

public class OffsetSwerveTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  public SwerveModuleState getState(double degrees, double offsetDegrees) {
    return new SwerveModuleState(
        1.0, Rotation2d.fromDegrees(degrees).plus(Rotation2d.fromDegrees(offsetDegrees)));
  }

  public void turningTest(
      Rotation2d moduleOffsetRads,
      Rotation2d currentPosition,
      Rotation2d desiredPositionRads,
      Rotation2d expectedPosition,
      double expectedMotorDirection) {
    // REVSwerveModule module = new REVSwerveModule(9, 10, moduleOffsetRads.getRadians());

    // TODO: Fix below line once sim is supported
    // module.simSetTurningMotorPosition(currentPosition);
    // module.setDesiredState(new SwerveModuleState(1.0, desiredPositionRads));

    // assertEquals(
    //     expectedPosition.getRadians(),
    //     module.getDesiredState().angle.getRadians(),
    //     kEpsilon,
    //     "Motor rotation incorrect");
    // assertEquals(
    //     Math.signum(expectedMotorDirection),
    //     Math.signum(module.getDesiredState().speedMetersPerSecond),
    //     "Motor direction inverted");

    // module.close();
  }

  @Test
  public void basicOffsetOptimize() {
    /**
     * With the front left module (rotated positive 90) and a sensor reading from the spark max of
     * 0.0 and a desired robot-relative swerve module position of 0.0 degrees the module must spin
     * either +90 and - motor, or -90 and + motor
     */
    turningTest(
        Rotation2d.fromRadians(Math.PI / 2), // Module offset
        Rotation2d.fromRadians(-0.1), // Motor Position (Spark Max relative)
        Rotation2d.fromRadians(0.0), // Desired position (Swerve relative)
        Rotation2d.fromRadians(-Math.PI / 2), // Expected position (Spark Max relative)
        1.0 // Motor direction
        );

    /**
     * With the front left module (rotated positive 90) and a sensor reading from the spark max of
     * -45 and a desired robot-relative swerve module position of 0.0 degrees the module must turn
     * to -90 and the motor should drive positive
     */
    turningTest(
        Rotation2d.fromRadians(Math.PI / 2), // Module offset
        Rotation2d.fromRadians(-Math.PI / 4), // Motor Position (Spark Max relative)
        Rotation2d.fromRadians(0.0), // Desired position (Swerve relative)
        Rotation2d.fromRadians(-Math.PI / 2), // Expected position (Spark Max relative)
        1.0 // Motor direction
        );

    /**
     * With the front left module (rotated positive 90) and a sensor reading from the spark max of
     * 45 and a desired robot-relative swerve module position of 0.0 degrees the module must turn to
     * 90 and the motor should drive negative
     */
    turningTest(
        Rotation2d.fromDegrees(90), // Module offset
        Rotation2d.fromDegrees(45), // Motor Position (Spark Max relative)
        Rotation2d.fromDegrees(0.0), // Desired position (Swerve relative)
        Rotation2d.fromDegrees(90), // Expected position (Spark Max relative)
        -1.0 // Motor direction
        );

    /**
     * With the rear left module (rotated positive 180) and a sensor reading from the spark max of 0
     * and a desired robot-relative swerve module position of 0.0 degrees the module should not move
     * and the motor should drive negative
     */
    turningTest(
        Rotation2d.fromRadians(Math.PI), // Module offset
        Rotation2d.fromRadians(0.0), // Motor Position (Spark Max relative)
        Rotation2d.fromRadians(0.0), // Desired position (Swerve relative)
        Rotation2d.fromRadians(0.0), // Expected position (Spark Max relative)
        -1.0 // Motor direction
        );

    /**
     * With the rear left module (rotated positive -180) and a sensor reading from the spark max of
     * 0 and a desired robot-relative swerve module position of 0.0 degrees the module should not
     * move and the motor should drive negative
     */
    turningTest(
        Rotation2d.fromRadians(-Math.PI), // Module offset
        Rotation2d.fromRadians(0.0), // Motor Position (Spark Max relative)
        Rotation2d.fromRadians(0.0), // Desired position (Swerve relative)
        Rotation2d.fromRadians(0.0), // Expected position (Spark Max relative)
        -1.0 // Motor direction
        );

    /**
     * With the rear left module (rotated positive 180) and a sensor reading from the spark max of
     * -179 and a desired robot-relative swerve module position of 90 degrees the module should not
     * move and the motor should drive negative
     */
    turningTest(
        Rotation2d.fromDegrees(180.0), // Module offset
        Rotation2d.fromDegrees(-179.0), // Motor Position (Spark Max relative)
        Rotation2d.fromDegrees(90.0), // Desired position (Swerve relative)
        Rotation2d.fromDegrees(-90.0), // Expected position (Spark Max relative)
        1.0 // Motor direction
        );
  }

  @Test
  public void rotationTest() {
    assertEquals(
        -90.0, Rotation2d.fromDegrees(270).plus(Rotation2d.fromDegrees(0)).getDegrees(), kEpsilon);
    assertEquals(
        175.0,
        Rotation2d.fromDegrees(-90).minus(Rotation2d.fromDegrees(95)).getDegrees(),
        kEpsilon);
    assertEquals(
        -175.0, Rotation2d.fromDegrees(90).plus(Rotation2d.fromDegrees(95)).getDegrees(), kEpsilon);
  }
}
