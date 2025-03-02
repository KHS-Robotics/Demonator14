// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

public interface ElevatorPID {
  double calculate(double measurement, double goal);
  void reset(double measurement);
}
