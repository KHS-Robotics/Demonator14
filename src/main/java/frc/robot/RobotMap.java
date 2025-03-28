/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {
  // Joysticks
  public static final int XBOX_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;
  
  // Swerve pivot motors
  public static final int FRONT_LEFT_PIVOT = 10;
  public static final int FRONT_RIGHT_PIVOT = 8;
  public static final int REAR_LEFT_PIVOT = 18;
  public static final int REAR_RIGHT_PIVOT = 20;

  // Swerve drive motors
  public static final int FRONT_LEFT_DRIVE = 11;
  public static final int FRONT_RIGHT_DRIVE = 9;
  public static final int REAR_LEFT_DRIVE = 19;
  public static final int REAR_RIGHT_DRIVE = 1;

  // Swerve pivot encoders
  public static final int FRONT_LEFT_PIVOT_ENCODER = 12;
  public static final int FRONT_RIGHT_PIVOT_ENCODER = 22;
  public static final int REAR_LEFT_PIVOT_ENCODER = 32;
  public static final int REAR_RIGHT_PIVOT_ENCODER = 42;

  // Elevator
  public static final int ELEVATOR_DRIVE_LEADER_ID = 12;
  public static final int ELEVATOR_DRIVE_FOLLOWER_ID = 7;

  // Coral Mechanism
  public static final int CORALLER_ANGLE_ID = 14;
  public static final int CORALLER_INTAKE_MOTOR_ID = 15;

  // Algae
  public static final int ALGAE_INTAKE_MOTOR_ID = 17;
  public static final int ALGAE_WRIST_MOTOR_ID = 2;

  // Climber
  public static final int CLIMBER_REEL_ID = 5;
  public static final int CLIMBER_ANCHOR_ID = 4;

  // LEDs over PWM
  public static final int LED_PORT = 0;
}