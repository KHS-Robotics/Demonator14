// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Thrustmaster T16000M
 */
public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  public Trigger stow() {
    return new Trigger(() -> this.getRawButton(ButtonMap.STOW_BUTTON));
  }

  public Trigger receive() {
    return new Trigger(() -> this.getRawButton(ButtonMap.RECEIVE_BUTTON));
  }

  public Trigger scoreL1() {
    return new Trigger(() -> this.getRawButton(ButtonMap.L1_BUTTON));
  }

  public Trigger scoreL2() {
    return new Trigger(() -> this.getRawButton(ButtonMap.L2_BUTTON));
  }

  public Trigger scoreL3() {
    return new Trigger(() -> this.getRawButton(ButtonMap.L3_BUTTON));
  }

  public Trigger scoreL4() {
    return new Trigger(() -> this.getRawButton(ButtonMap.L4_BUTTON));
  }

  public Trigger outtakeCoral() {
    return new Trigger(() -> this.getRawButton(ButtonMap.OUTTAKE_BUTTON));
  }

  public Trigger intakeCoral() {
    return new Trigger(() -> this.getRawButton(ButtonMap.INTAKE_BUTTON));
  }

  public Trigger reelInClimber() {
    return new Trigger(() -> this.getRawButton(ButtonMap.REEL_IN_BUTTON));
  }

  public Trigger reelOutClimber() {
    return new Trigger(() -> this.getRawButton(ButtonMap.REEL_OUT_BUTTON));
  }

  public Trigger engageAnchor() {
    return new Trigger(() -> this.getRawButton(ButtonMap.ENGAGEANCHOR_BUTTON));
  }

  public Trigger disengageAnchor() {
    return new Trigger(() -> this.getRawButton(ButtonMap.UNENGAGEANCHOR_BUTTON));
  }
}
