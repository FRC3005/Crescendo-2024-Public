package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.geometry.Rotation2d;

public record TargetResult(double distance, Rotation2d rotation, boolean inView) {
  public TargetResult() {
    this(0.0, new Rotation2d(), false);
  }
}
