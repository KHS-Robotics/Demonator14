// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;

/** Add your docs here. */
public class ElevatorPIDBase implements ElevatorPID {
  private final PIDController pid;

  public ElevatorPIDBase() {
    pid = new PIDController(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD);
    pid.setIZone(3);
  }

  public double calculate(double measurement, double goal) {
    return pid.calculate(measurement, goal);
  }

  public void reset(double mesaurement) {
    pid.reset();
  }
}
