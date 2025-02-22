// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Demonator14 for FRC Reefscape 2025.
 * 
 * <p>
 * 
 * https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html#timedrobot
 * 
 * <p>
 * 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html
 */
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /**
   * Creates and initializes Demonator14.
   * 
   * <p>
   * 
   * https://www.thebluealliance.com/team/4342/2025
   */
  public Robot() {
    // silence disconnected joystick warnings when not FMS attached
    DriverStation.silenceJoystickConnectionWarning(true);

    // for debugging
    CommandScheduler.getInstance().onCommandInitialize((cmd) -> System.out.println(cmd.getName() + " started."));
    CommandScheduler.getInstance().onCommandInterrupt((cmd) -> System.out.println(cmd.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish((cmd) -> System.out.println(cmd.getName() + " ended."));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated
   * and test.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html
    CommandScheduler.getInstance().run();

    // Update the robot's position and heading on the Dashboard GUI
    RobotContainer.kField.setRobotPose(RobotContainer.kSwerveDrive.getPose());
    
    // Joystick connection info
    SmartDashboard.putBoolean("Joysticks/Xbox Controller", RobotContainer.kDriverController.isConnected());
    SmartDashboard.putBoolean("Joysticks/Operator Stick", RobotContainer.kOperatorStick.isConnected());
  }

  /** This method is called once each time the robot enters disabled mode. */
  @Override
  public void disabledInit() {
  }

  /** This method is called periodically during disabled mode. */
  @Override
  public void disabledPeriodic() {
  }

  /** This method is called once each time the robot exits disabled mode. */
  @Override
  public void disabledExit() {
  }

  /** This method is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // Get selected auto from SmartDashboard
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html#starting-an-autonomous-command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if the selected autonomous routine exists, run it
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // disable AprilTag updates for auton: potentially temporary and may remove this later
    RobotContainer.kLowerFrontPhotonCamera.setEnableAprilTagUpdates(false);
    RobotContainer.kRearLimelightCamera.setEnableAprilTagUpdates(false);
  }

  /** This method is called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This method is called once each time the robot exits autonomous mode. */
  @Override
  public void autonomousExit() {
    RobotContainer.kLowerFrontPhotonCamera.setEnableAprilTagUpdates(true);
    RobotContainer.kRearLimelightCamera.setEnableAprilTagUpdates(true);
  }

  /**
   * This method is called once each time the robot enters teleop mode.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    // If you want the autonomous to continue until interrupted by another command,
    // remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This method is called periodically during teleop mode. */
  @Override
  public void teleopPeriodic() {
  }

  /** This method is called once each time the robot exits teleop mode. */
  @Override
  public void teleopExit() {
  }

  /** This method is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This method is called once each time the robot exits test mode. */
  @Override
  public void testExit() {
  }

  /**
   * This method is called once when the robot is first started up in simulation.
   */
  @Override
  public void simulationInit() {
  }

  /** This method is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
