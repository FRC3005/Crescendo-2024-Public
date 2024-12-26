package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.odometry.CarpetTransform;
import frc.lib.odometry.SwerveDriveCarpetKinematics;
import frc.lib.swerve.ModuleLimits;

public final class DrivetrainConstants {
  // Drive motor current limit in Amps
  public static final int kDriveMotorCurrentLimit = 55;
  public static final int kDriveMotorCurrentChop = 70;

  // Drive motor current limit in Amps
  public static final int kDriveMotorCurrentLimitAuton = 85;

  // Max Velocity in m/s
  public static final double kTurningMotorMaxVelocity = 5.0;

  public static PIDGains kDriveMotorPIDGains = new PIDGains(0.186, 0.0, 0.001);
  public static final PIDGains kTurningMotorPIDGains = new PIDGains(2.138, 0.00, 0.2);

  public static final double kTrackWidth = Units.inchesToMeters(22.5);
  public static final double kWheelBase = Units.inchesToMeters(22.5);
  public static final Translation2d[] kModuleLocations =
      new Translation2d[] {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      };

  // Max acceleration in m/s^2
  public static final double kTurningMotorMaxAccel = 5.0;
  public static final double kMaxDriveSpeed = 4.8;
  public static final double kMaxDriveAcceleration = 5.5;
  public static final double kMaxAngularVelocity = 11.5;
  // 2.0 * (kMaxDriveSpeed / kModuleLocations[0].getDistance(kModuleLocations[3]));

  public static ModuleLimits kModuleLimits =
      new ModuleLimits(kMaxDriveSpeed, kMaxDriveSpeed / 0.01, Units.degreesToRadians(2500.0));

  // Number of rotations of the motor / number of rotations of the output
  public static final double kDriveMotorReduction = (20.0 / 14.0) * 3.0;
  public static final double kWheelDiameterMeters = Units.inchesToMeters(2.91);

  // Assumes the encoders are directly mounted on the wheel shafts
  public static final double kDriveEncoderPositionFactor =
      // (1.0) / kDriveMotorReduction;
      (kWheelDiameterMeters * Math.PI) / kDriveMotorReduction;

  // public static final double kDriveEncoderPositionFactor = 1.0 / kDriveMotorReduction;

  public static final double kDriveEncoderVelocityFactor =
      // Assumes the encoders are directly mounted on the wheel shafts
      ((kWheelDiameterMeters * Math.PI) / (double) kDriveMotorReduction) / 60.0;
  public static final boolean kDriveMotorInvert = false;

  public static final double kTurningModuleGearRatio = 9424.0 / 203.0;
  public static final double kTurningEncoderPositionFactor =
      (2 * Math.PI) / kTurningModuleGearRatio;
  public static final double kTurningEncoderVelocityFactor =
      ((2 * Math.PI) / kTurningModuleGearRatio) / 60.0;
  public static final boolean kTurningMotorInvert = false;
  public static final SimpleMotorFeedforward kDriveFeedforward =
      new SimpleMotorFeedforward(0.17, 2.15, 0.30895);
  public static final SimpleMotorFeedforward kTurningFeedforward =
      new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);
  public static final TrapezoidProfile.Constraints kTurningConstraints =
      new TrapezoidProfile.Constraints(20, 200);

  public static final int kFrontLeftAbsoluteEncoderPort = 0;
  public static final int kFrontRightAbsoluteEncoderPort = 3;
  public static final int kRearLeftAbsoluteEncoderPort = 1;
  public static final int kRearRightAbsoluteEncoderPort = 2;

  // Auton path finding controllers
  public static final double kTranslationP = 5.0;
  public static final double kTranslationD = 0.0;
  public static final PIDController kXController =
      new PIDController(kTranslationP, 0.0, kTranslationD);
  public static final PIDController kYController =
      new PIDController(kTranslationP, 0.0, kTranslationD);

  // High profile constraints = pure P controller
  public static final double kRotationP = 5.0;
  public static final double kRotationD = 0.0;
  public static PIDController kThetaController = new PIDController(kRotationP, 0.0, kRotationD);
  public static DCMotor kDriveMotorSim = DCMotor.getNeoVortex(1);
  public static DCMotor kTurningMotorSim = DCMotor.getNeo550(1);

  public static final HolonomicPathFollowerConfig kPathPlannerConfig =
      new HolonomicPathFollowerConfig(
          new PIDConstants(kTranslationP, 0.0, kTranslationD),
          new PIDConstants(kTranslationP, 0.0, kTranslationD),
          kMaxDriveSpeed,
          kModuleLocations[0].getNorm(),
          new ReplanningConfig(false, false));

  private static final CarpetTransform kCarpetTransform =
      new CarpetTransform(0.98, 1.02, Field.kCompetitionCarpet);

  public static final boolean kEnableCarpetCorrection = false;
  public static final SwerveDriveCarpetKinematics kDriveCarpetKinematics =
      new SwerveDriveCarpetKinematics(kCarpetTransform, kModuleLocations);
  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(kModuleLocations);
}
