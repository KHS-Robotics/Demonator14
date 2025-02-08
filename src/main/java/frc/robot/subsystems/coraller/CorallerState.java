// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

import frc.robot.subsystems.coraller.CorallerSetpoints.AnglerSetpoints;
import frc.robot.subsystems.coraller.CorallerSetpoints.ElevatorSetpoints;
import frc.robot.subsystems.coraller.CorallerSetpoints.FlickerSetpoints;

/** Heights and angles for the Coraller's possible states. */
enum CorallerState {
  STOW(ElevatorSetpoints.STOW_HEIGHT, AnglerSetpoints.STOW_ANGLE, FlickerSetpoints.STOW_ANGLE),
  RECEIVE(ElevatorSetpoints.RECEIVE_HEIGHT, AnglerSetpoints.RECEIVE_ANGLE, FlickerSetpoints.STOW_ANGLE),
  L1(ElevatorSetpoints.L1_HEIGHT, AnglerSetpoints.L1_ANGLE, FlickerSetpoints.STOW_ANGLE),
  L2(ElevatorSetpoints.L2_HEIGHT, AnglerSetpoints.L2_ANGLE, FlickerSetpoints.STOW_ANGLE),
  L2_ALGAE(ElevatorSetpoints.L2_ALGAE_HEIGHT, AnglerSetpoints.L2_ALGAE_ANGLE, FlickerSetpoints.L2_ANGLE),
  L3(ElevatorSetpoints.L3_HEIGHT, AnglerSetpoints.L3_ANGLE, FlickerSetpoints.STOW_ANGLE),
  L3_ALGAE(ElevatorSetpoints.L3_ALGAE_HEIGHT, AnglerSetpoints.L3_ALGAE_ANGLE, FlickerSetpoints.L3_ANGLE),
  L4(ElevatorSetpoints.L4_HEIGHT, AnglerSetpoints.L4_ANGLE, FlickerSetpoints.STOW_ANGLE);

  /** Inches */
  public final double elevatorPosition;
  /** Degrees */
  public final double anglerPosition;
  /** Degrees */
  public final double flickerPosition;

  private CorallerState(double elevatorPosition, double anglerPosition, double flickerPosition) {
    this.elevatorPosition = elevatorPosition;
    this.anglerPosition = anglerPosition;
    this.flickerPosition = flickerPosition;
  }
}
