/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Default command for swerve drive.
 */
public class DriveSwerveWithXbox extends Command {
  /** Deadband since joysticks vary in how well they snap back to zero. */
  private static final double kJoystickDeadband = 0.035;

  private final BooleanSupplier fieldRelative;
  private final DoubleSupplier joystickSensitivity;

  /**
   * Control the swerve drive with an Xbox controller.
   * 
   * @param fod                 field oriented drive
   * @param joystickSensitivity Ranges from [0, 1] where 0 is full linear and 1 is
   *                            full cubic.
   */
  public DriveSwerveWithXbox(BooleanSupplier fod, DoubleSupplier joystickSensitivity) {
    this.addRequirements(RobotContainer.kSwerveDrive);
    this.fieldRelative = fod;
    this.joystickSensitivity = joystickSensitivity;
  }

  /**
   * Control the swerve drive with an Xbox controller.
   * 
   * @param fod                 field oriented drive
   * @param joystickSensitivity Ranges from [0, 1] where 0 is full linear and 1 is
   *                            full cubic.
   */
  public DriveSwerveWithXbox(boolean fod, double joystickSensitivity) {
    this(() -> fod, () -> joystickSensitivity);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = 0.0;
    if (Math.abs(RobotContainer.kDriverController.getLeftY()) > kJoystickDeadband) {
      xSpeed = sensControl(-RobotContainer.kDriverController.getLeftY())
          * SwerveDrive.maxSpeedMetersPerSecond;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = 0.0;
    if (Math.abs(RobotContainer.kDriverController.getLeftX()) > kJoystickDeadband) {
      ySpeed = sensControl(-RobotContainer.kDriverController.getLeftX())
          * SwerveDrive.maxSpeedMetersPerSecond;
    }

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = 0;
    if (Math.abs(RobotContainer.kDriverController.getRightX()) > kJoystickDeadband) {
      rot = sensControl(-RobotContainer.kDriverController.getRightX())
          * SwerveDrive.maxAngularSpeedRadiansPerSecond;
    }

    var sign = fieldRelative.getAsBoolean() && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    RobotContainer.kSwerveDrive.drive(sign * xSpeed, sign * ySpeed, rot, fieldRelative.getAsBoolean());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.kSwerveDrive.stop();
  }

  /**
   * Sensitivity control for the joystick that uses a cubic function to smooth out
   * the inputs
   * instead of linear control: <code> s*x^3 + (1-s)*x </code> where s is the
   * sensitivity and x is the input.
   * https://www.wolframalpha.com/input?i=0.5%3A+s*x%5E3+%2B+%281-s%29*x+where+s+%3D+0.5
   * 
   * @param s the sensitivity from [0, 1] where 0 is full linear and 1 is full
   *          cubic
   * @return the new dampened control value
   */
  private double sensControl(double s) {
    return joystickSensitivity.getAsDouble() * Math.pow(s, 3) + (1 - joystickSensitivity.getAsDouble()) * s;
  }
}
