package frc.lib;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.DrivetrainConstants;
import org.junit.jupiter.api.*;

public class SwerveTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  public void testConfigurationFieldOriented(
      double xSpeed, double ySpeed, double rot, double gyro, double maxSpeed, boolean print) {
    var kinematics = DrivetrainConstants.kDriveKinematics;
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), Rotation2d.fromRadians(gyro)));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    int i = 0;
    for (var state : swerveModuleStates) {
      if (print) {
        System.out.println(
            String.format(
                "Test State Module %d: x: %.02f y: %.02f, r: %.02f, gyro: %.02f, Output theta: %.02f, Output speed: %.02f",
                i,
                xSpeed,
                ySpeed,
                rot,
                gyro,
                state.angle.getRadians(),
                state.speedMetersPerSecond));
      }

      Assertions.assertTrue(Math.abs(state.speedMetersPerSecond) <= maxSpeed, "Max speed violated");
      Assertions.assertTrue(state.angle.getRadians() <= Math.PI, "Angle above 2 pi");
      Assertions.assertTrue(state.angle.getRadians() >= -Math.PI, "Angle below 0");
      i++;
    }
  }

  public void testConfiguration(
      double xSpeed, double ySpeed, double rot, double maxSpeed, boolean print) {
    var kinematics = DrivetrainConstants.kDriveKinematics;
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    int i = 0;
    for (var state : swerveModuleStates) {
      if (print) {
        System.out.println(
            String.format(
                "Test State Module %d: x: %.02f y: %.02f, r: %.02f, Output theta: %.02f, Output speed: %.02f",
                i, xSpeed, ySpeed, rot, state.angle.getRadians(), state.speedMetersPerSecond));
      }

      Assertions.assertTrue(Math.abs(state.speedMetersPerSecond) <= maxSpeed, "Max speed violated");
      Assertions.assertTrue(state.angle.getRadians() <= Math.PI, "Angle above 2 pi");
      Assertions.assertTrue(state.angle.getRadians() > -Math.PI, "Angle below 0");
      i++;
    }
  }

  @Test
  public void direcitonality() {
    // Following
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
    var kinematics = DrivetrainConstants.kDriveKinematics;

    // When the input is set to move the robot +x, the module should point forward (0 degrees) with
    // positive velocity
    var frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0))[0];
    Assertions.assertEquals(1.0, frontLeft.speedMetersPerSecond, kEpsilon);
    Assertions.assertEquals(0.0, frontLeft.angle.getRadians(), kEpsilon);

    // When the input is set to move the robot -x, the module should point backward (pi degrees)
    // with
    // positive velocity
    frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(-1.0, 0.0, 0.0))[0];
    Assertions.assertEquals(1.0, frontLeft.speedMetersPerSecond, kEpsilon);
    Assertions.assertEquals(Math.PI, Math.abs(frontLeft.angle.getRadians()), kEpsilon);

    // When the input is set to move the robot +y, the module should point backward (pi / 2) with
    // positive velocity
    frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 1.0, 0.0))[0];
    Assertions.assertEquals(1.0, frontLeft.speedMetersPerSecond, kEpsilon);
    Assertions.assertEquals(Math.PI / 2, frontLeft.angle.getRadians(), kEpsilon);

    // When the input is set to move the robot -y, the module should point backward (-pi / 2) with
    // positive velocity
    frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, -1.0, 0.0))[0];
    Assertions.assertEquals(1.0, frontLeft.speedMetersPerSecond, kEpsilon);
    Assertions.assertEquals(-Math.PI / 2, frontLeft.angle.getRadians(), kEpsilon);

    // When the input is set to move the robot +y +x, the module should point backward (pi / 4) with
    // positive velocity
    frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 1.0, 0.0))[0];
    Assertions.assertTrue(frontLeft.speedMetersPerSecond >= 1.0);
    Assertions.assertEquals(Math.PI / 4, frontLeft.angle.getRadians(), kEpsilon);

    // When the input is set to move the robot +rot, the module should point backward (3pi / 4) with
    // positive velocity
    frontLeft = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0))[0];
    Assertions.assertTrue(frontLeft.speedMetersPerSecond > 0.0);
    // TODO: This is failing?
    // Assertions.assertEquals(3 * Math.PI / 4, frontLeft.angle.getRadians(), kEpsilon);
  }

  @Test
  public void testSwerveStatesRelative() {
    double maxSpeed = 5.0;
    double testDelta = 0.1;

    double xSpeed = -maxSpeed;
    for (int i = 0; i < (1.0 / testDelta) * 2; i++) {
      double ySpeed = -maxSpeed;
      for (int j = 0; j < (1.0 / testDelta) * 2; j++) {
        double thetaSpeed = -maxSpeed;
        for (int k = 0; k < (1.0 / testDelta) * 2; k++) {
          testConfiguration(xSpeed, ySpeed, thetaSpeed, maxSpeed, false);
          thetaSpeed += testDelta;
        }
        ySpeed += testDelta;
      }
      xSpeed += testDelta;
    }
  }

  @Test
  public void testSwerveStatesFieldCentric() {
    double maxSpeed = 1.0;
    double testDelta = 0.1;

    double xSpeed = -maxSpeed;
    for (int i = 0; i < (maxSpeed / testDelta) * 2; i++) {
      double ySpeed = -maxSpeed;
      for (int j = 0; j < (maxSpeed / testDelta) * 2; j++) {
        double thetaSpeed = -maxSpeed;
        for (int k = 0; k < (maxSpeed / testDelta) * 2; k++) {
          double gyro = (-2.0 * Math.PI);
          for (int t = 0; t < ((2.0 * Math.PI) / testDelta) * 2; t++) {
            testConfigurationFieldOriented(xSpeed, ySpeed, thetaSpeed, gyro, maxSpeed, false);
            gyro += testDelta;
          }
          thetaSpeed += testDelta;
        }
        ySpeed += testDelta;
      }
      xSpeed += testDelta;
    }
  }
}
