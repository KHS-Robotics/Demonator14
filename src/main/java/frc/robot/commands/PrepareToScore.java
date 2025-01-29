// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coraller.Level;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareToScore extends Command {

  private final Level level;

  public PrepareToScore(Level level) {
    addRequirements(RobotContainer.kCoraller);
    this.level = level;
  }

  @Override
  public void initialize() {
    RobotContainer.kCoraller.setPosition(level);
  }
}
