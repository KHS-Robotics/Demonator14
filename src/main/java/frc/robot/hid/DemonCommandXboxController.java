// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}.
 * 
 * @see edu.wpi.first.wpilibj2.command.button.CommandXboxController
 */
public class DemonCommandXboxController extends CommandXboxController {
  public DemonCommandXboxController(int port) {
    super(port);
  }

  public Trigger isPressingResetRobotHeading() {
    return this.start().debounce(0.5);
  }
}
