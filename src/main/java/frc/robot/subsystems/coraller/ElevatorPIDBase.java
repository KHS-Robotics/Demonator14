// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;

public class ElevatorPIDBase extends PIDController implements ElevatorPID {
  public ElevatorPIDBase() {
    super(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD);
    this.setIZone(3);
  }

  @Override
  public void reset(double mesaurement) {
    this.reset();
  }
}
