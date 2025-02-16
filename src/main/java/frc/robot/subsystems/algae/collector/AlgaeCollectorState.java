// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae.collector;

import frc.robot.subsystems.algae.collector.AlgaeCollectorSetpoints.WristSetpoints;

enum AlgaeCollectorState {
  STOW(WristSetpoints.STOW),
  CLIMB(WristSetpoints.CLIMB),
  DEPLOY(WristSetpoints.DEPLOY);

  public final double wristAngle;

  private AlgaeCollectorState(double wristAngle) {
    this.wristAngle = wristAngle;
  }
}
