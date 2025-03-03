// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;

public class ElevatorPIDTrapazoid extends ProfiledPIDController implements ElevatorPID {
  public ElevatorPIDTrapazoid() {
    super(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD,
      new TrapezoidProfile.Constraints(ElevatorConfig.kElevatorMaxVelocity, ElevatorConfig.kElevatorMaxAcceleration));
  }
}
