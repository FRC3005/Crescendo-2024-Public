// MIT License

// Copyright (c) 2023 Team 254

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.EqualsUtil;
import frc.lib.util.GeomUtil;
import org.junit.jupiter.api.Test;

public class SwerveSetpointGeneratorTest {
  protected static final double kRobotSide = 0.616; // m

  private static final Translation2d[] moduleLocations =
      new Translation2d[] {
        // Front left
        new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
        // Front right
        new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
        // Back left
        new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
        // Back right
        new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0)
      };

  protected static final SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(moduleLocations);
  protected static final ModuleLimits kKinematicLimits =
      new ModuleLimits(5.0, 10.0, Math.toRadians(1500.0));

  protected static final double kDt = 0.01; // s
  protected static final double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
  protected static final double kMaxAccelerationError = 0.01; // m/s^2

  public void SatisfiesConstraints(
      SwerveSetpoint prev, SwerveSetpoint next, boolean checkAcceleration) {
    for (int i = 0; i < prev.moduleStates().length; ++i) {
      final var prevModule = prev.moduleStates()[i];
      final var nextModule = next.moduleStates()[i];
      Rotation2d diffRotation = GeomUtil.inverse(prevModule.angle).rotateBy(nextModule.angle);
      assertTrue(
          Math.abs(diffRotation.getRadians())
              < kKinematicLimits.maxSteeringVelocity() + kMaxSteeringVelocityError);
      assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.maxDriveVelocity());
      assertTrue(
          Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt
              <= kKinematicLimits.maxDriveAcceleration() + kMaxAccelerationError);

      if (checkAcceleration) {
        // If we should check acceleration, check that we are reaching max acceleration at all
        // times.
        assertEquals(
            Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt,
            kKinematicLimits.maxDriveAcceleration(),
            kMaxAccelerationError);
      }
    }
  }

  public SwerveSetpoint driveToGoal(
      SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
    return driveToGoal(prevSetpoint, goal, generator, false);
  }

  public SwerveSetpoint driveToGoal(
      SwerveSetpoint prevSetpoint,
      ChassisSpeeds goal,
      SwerveSetpointGenerator generator,
      boolean checkAcceleration) {
    System.out.println("Driving to goal state " + goal);
    System.out.println("Initial state: " + prevSetpoint);
    while (!EqualsUtil.GeomExtensions.epsilonEquals(
        GeomUtil.toTwist2d(prevSetpoint.chassisSpeeds()), GeomUtil.toTwist2d(goal))) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
      System.out.println(newsetpoint);
      SatisfiesConstraints(prevSetpoint, newsetpoint, checkAcceleration);
      prevSetpoint = newsetpoint;
    }
    return prevSetpoint;
  }

  @Test
  public void testAccelerationLimits() {
    SwerveModuleState[] initialStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

    var generator = new SwerveSetpointGenerator(kKinematics, moduleLocations);

    // Just drive straight
    var goalSpeeds = new ChassisSpeeds(5.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator, true);
  }

  @Test
  public void testGenerateSetpoint() {
    SwerveModuleState[] initialStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

    var generator = new SwerveSetpointGenerator(kKinematics, moduleLocations);

    var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);
  }
}
