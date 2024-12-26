// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainer {

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand();

  public void logPeriodic();

  public void disabledInit();

  public void disabledPeriodic();

  public void autonInit();

  public void teleopInit();

  public void testModeInit();

  public void testModePeriodic();
}
