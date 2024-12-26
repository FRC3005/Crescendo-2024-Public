package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.DoubleSupplier;

public interface SetpointTracking {
  public void setTrackingFunction(DoubleSupplier trackingSupplier);

  public void enableTracking();

  public void disableTracking();

  public boolean isTrackingEnabled();

  public default Command enableTrackingCommand() {
    return new InstantCommand(() -> enableTracking());
  }

  public default Command enableTrackingCommand(DoubleSupplier trackingSupplier) {
    return new InstantCommand(
        () -> {
          setTrackingFunction(trackingSupplier);
          enableTracking();
        });
  }

  public default Command disableTrackingCommand() {
    return new InstantCommand(() -> disableTracking());
  }
}
