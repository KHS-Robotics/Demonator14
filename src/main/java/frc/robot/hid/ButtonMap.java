// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

public class ButtonMap {
  public static class POV {
    public static final int OUTTAKE_L4 = 0;
  }

  // Elevator + Angler
  public static final int STOW_BUTTON = 8;
  public static final int RECEIVE_BUTTON = 7;
  public static final int L1_BUTTON = 9;
  public static final int L2_BUTTON = 10;
  public static final int L3_BUTTON = 6;
  public static final int L4_BUTTON = 5;

  // Coral Intake
  public static final int CORAL_OUTTAKE_BUTTON = 1;
  public static final int CORAL_INTAKE_BUTTON = 2;

  // Algae Intake
  public static final int ALGAE_OUTTAKE_BUTTON = 4;
  public static final int ALGAE_INTAKE_BUTTON = 3;
  public static final int ALGAE_STOW_BUTTON = 14;
  public static final int ALGAE_DEPLOY_BUTTON = 13;

  // Climber
  public static final int REEL_IN_BUTTON = 16;
  public static final int REEL_OUT_BUTTON = 11;
  public static final int ENGAGEANCHOR_BUTTON = 15;
  public static final int UNENGAGEANCHOR_BUTTON = 12;
}
