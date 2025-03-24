// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Thrustmaster T16000M
 */
public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  // Elevator + Angler

  public Trigger disableElevatorOverride() {
    var stow = stowCoraller();
    BooleanSupplier disableOverride = () -> getPOV() == ButtonMap.POV.DISABLE_ELEVATOR_OVERRIDE;
    var disableElevatorOverride = stow.and(disableOverride);
    return disableElevatorOverride.debounce(0.5);
  }

  public Trigger enableElevatorOverride() {
    var stow = stowCoraller();
    BooleanSupplier enableOverride = () -> getPOV() == ButtonMap.POV.ENABLE_ELEVATOR_OVERRIDE;
    var enableElevatorOverride = stow.and(enableOverride);
    return enableElevatorOverride.debounce(0.5);
  }

  public Trigger stowCoraller() {
    return new Trigger(() -> this.getRawButton(ButtonMap.STOW_BUTTON));
  }

  public Trigger receiveCoraller() {
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

  // Coral Intake

  public Trigger outtakeForL4() {
    return new Trigger(() -> this.getPOV() == ButtonMap.POV.OUTTAKE_L4);
  }

  public Trigger outtakeCoral() {
    return new Trigger(() -> this.getRawButton(ButtonMap.CORAL_OUTTAKE_BUTTON));
  }

  public Trigger intakeCoral() {
    return new Trigger(() -> this.getRawButton(ButtonMap.CORAL_INTAKE_BUTTON));
  }
  
  // Algae Intake
  
  public Trigger stowAlgaeCollector() {
    return new Trigger(() -> this.getRawButton(ButtonMap.ALGAE_STOW_BUTTON));
  }
  
  public Trigger deployAlgaeCollector() {
    return new Trigger(() -> this.getRawButton(ButtonMap.ALGAE_DEPLOY_BUTTON));
  }

  public Trigger intakeAlgae() {
    return new Trigger(() -> this.getRawButton(ButtonMap.ALGAE_INTAKE_BUTTON));
  }

  public Trigger outtakeAlgae() {
    return new Trigger(() -> this.getRawButton(ButtonMap.ALGAE_OUTTAKE_BUTTON));
  }

  // Climber

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
