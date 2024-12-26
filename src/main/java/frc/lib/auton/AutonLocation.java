package frc.lib.auton;

public abstract class AutonLocation {
  // public abstract Pose2d get();

  public String getName() {
    String name = this.getClass().getSimpleName();
    return name.substring(name.lastIndexOf('.') + 1);
  }
}
