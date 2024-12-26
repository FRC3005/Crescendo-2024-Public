package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;

public class ClimberConstants {
  public static final double kStartingPosition = 0.0;
  public static final double kTopPosition = 0.48;

  public static final double kGearReduction = 20.0 / 1.0; // Updated gearbox ratio
  public static final double kDrumDiameter = Units.inchesToMeters(1.25);

  public static final double kClimberMassKg = Units.lbsToKilograms(10.1223685);
  public static final DCMotor kClimbGearbox = DCMotor.getNEO(1);
  public static final boolean kInverted = true;
  public static final double kRetractSpeed = -1.0;
  public static final double kUnlockTimeSeconds = 1.0;
  public static final double kLockAfterClimbTimeSeconds = 0.75;
  public static PIDGains kPIDGains = new PIDGains(5.0, 0.0, 0.0);
}
