// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.telemetry;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.Sendable;

public interface TelemetryBuilder extends NTSendableBuilder {
  void bindChild(String name, TelemetryNode node);

  void bindSendableChild(String name, Sendable sendable);
}
