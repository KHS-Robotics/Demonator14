// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraller;

final class CorallerSetpoints {
  protected final class AnglerSetpoints {
    public static final double STOW_ANGLE = 65;
    public static final double L1_ANGLE = -40;
    public static final double L2_ANGLE = -35;
    public static final double L2_ALGAE_ANGLE = -65;
    public static final double L3_ANGLE = -35;
    public static final double L3_ALGAE_ANGLE = -65;
    public static final double L4_ANGLE = -65;
    public static final double RECEIVE_ANGLE = 35;
  }

  protected final class ElevatorSetpoints {
    // all in Inches
    public static final double STOW_HEIGHT = 34.625;
    // ^2.057 Tonys
    public static final double RECEIVE_HEIGHT = 34.625;
    public static final double L1_HEIGHT = 38;
    public static final double L2_HEIGHT = 43;
    public static final double L2_ALGAE_HEIGHT = 35;
    public static final double L3_HEIGHT = 59;
    public static final double L3_ALGAE_HEIGHT = 48;
    public static final double L4_HEIGHT = 81;
  }
}
