package frc.lib.util;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestUtils {
  /**
   * Advance a subsystem for some about of time during tests. This simply calls the subsystem
   * periodic functions some number of times. Also steps the internal FPGA timestamp, allowing any
   * notifiers to run.
   *
   * @param subsystem subsystem to advance
   * @param seconds number of seconds to advance
   * @param subsystem_dt dt of the subsystem scheduler
   */
  public static void subsystemTimeAdvance(
      double seconds, double subsystem_dt, Subsystem... subsystems) {
    int iterations = (int) Math.ceil(seconds / subsystem_dt);
    for (int i = 0; i < iterations; i++) {
      SimHooks.stepTiming(subsystem_dt);
      for (var subsystem : subsystems) {
        subsystem.periodic();
        subsystem.simulationPeriodic();
      }
    }
  }

  /**
   * Advance a subsystem for some about of time during tests. This simply calls the subsystem
   * periodic function some number of times.
   *
   * @param subsystem subsystem to advance
   * @param seconds number of seconds to advance
   */
  public static void subsystemTimeAdvance(double seconds, Subsystem... subsystems) {
    subsystemTimeAdvance(seconds, 0.02, subsystems);
  }
}
