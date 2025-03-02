// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;

/** Add your docs here. */
public class ElevatorPIDTrapazoid implements ElevatorPID {
  private final ProfiledPIDController pid;

  public ElevatorPIDTrapazoid() {
    pid = new ProfiledPIDController(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD,
      new TrapezoidProfile.Constraints(ElevatorConfig.kElevatorMaxVelocity, ElevatorConfig.kElevatorMaxAcceleration));
    pid.setIZone(3);

    SmartDashboard.putData(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName() + "/PID", pid);
  }

  public double calculate(double mesaurement, double goal) {
    return pid.calculate(mesaurement, goal);
  }

  public void reset(double measurement) {
    pid.reset(measurement);
  }
}
