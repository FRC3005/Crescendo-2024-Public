package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.util.RobotName;

public final class ArmConstants {
  ///////////////////////////////////////////
  // Per-robot tuning for through bore
  ///////////////////////////////////////////
  // kHardStopAngle: measure with an inclanometer against a frame member
  public static final double kHardStopAngle = Units.degreesToRadians(RobotName.select(17.8, 16.8));

  // kHardStopSensorAngle: set kThroughBoreOffset to 0 and read its output and enter it here:
  private static final double kHardStopSensorAngle =
      Units.degreesToRadians(RobotName.select(6.937983, 9.216362));
  ///////////////////////////////////////////
  // End
  ///////////////////////////////////////////

  public static final double kThroughBoreOffset = kHardStopAngle - kHardStopSensorAngle;

  // Set this value based on where we expect to power on the bot. This is a fallback incase
  // the through bore is unplugged.
  public static final double kStartingPosition = kHardStopAngle;

  public static final double kGearReduction = (66.0 / 10.0) * (66.0 / 18.0) * (90.0 / 16.0);
  public static final double kMOI = 2.2; // kg*m^2

  // 0.0 degrees = straight out, positive going up towards vertical
  public static final double kReverseSoftLimit = kHardStopAngle;
  public static final double kForwardSoftLimit =
      RobotName.select(Units.degreesToRadians(94.0), Units.degreesToRadians(91.3));
  public static final double kUnderStageLimit = Units.degreesToRadians(28.0);
  public static final boolean kMotorInvert = true;
  public static final PIDGains kGains = new PIDGains(20.0, 0.0, 0.5);
  public static final Constraints kConstraints = new Constraints(4.0, 8.0);
  public static final ArmFeedforward kFeedForward = new ArmFeedforward(0.0, 0.3, 0.0);
  public static final double kArmLengthM = Units.inchesToMeters(20.0);
  public static final double kMinRotation = kReverseSoftLimit;
  public static final double kMaxRotation = kForwardSoftLimit;
  public static final double kArmMassKg = Units.lbsToKilograms(33.0);
  public static final DCMotor kArmGearbox = DCMotor.getNEO(1);
}
