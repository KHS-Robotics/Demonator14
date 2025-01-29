// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coraller.CorallerPosition;
import frc.robot.subsystems.Coraller.ElevatorPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareToScore extends Command {
  public enum Level {
    STOW(ElevatorPosition.kStow, CorallerPosition.kStow),
    L1(ElevatorPosition.kLevel1, CorallerPosition.kLevel1),
    L2(ElevatorPosition.kLevel2, CorallerPosition.kLevel2),
    L3(ElevatorPosition.kLevel3, CorallerPosition.kLevel3),
    L4(ElevatorPosition.kLevel4, CorallerPosition.kLevel4);

    private final ElevatorPosition elevatorPosition;
    private final CorallerPosition corallerPosition;

    private Level(ElevatorPosition elevatorPosition, CorallerPosition corallerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.corallerPosition = corallerPosition;
    }
  }

  private final Level level;

  public PrepareToScore(Level level) {
    addRequirements(RobotContainer.kCoraller);
    this.level = level;
  }

  


  @Override
  public void initialize() {
    RobotContainer.kCoraller.setCorallerPosition(level.corallerPosition);
    RobotContainer.kCoraller.
  }
}
