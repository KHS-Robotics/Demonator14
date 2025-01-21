// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html#timedrobot
 */
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // for debugging
    CommandScheduler.getInstance()
        .onCommandInitialize((command) -> System.out.println(command.getName() + " started."));
    CommandScheduler.getInstance()
        .onCommandInterrupt((command) -> System.out.println(command.getName() + " interrupted."));
    CommandScheduler.getInstance()
        .onCommandFinish((command) -> System.out.println(command.getName() + " ended."));

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    // Get selected auto from SmartDashboard
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html#starting-an-autonomous-command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
