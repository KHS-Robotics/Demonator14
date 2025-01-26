// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  public boolean isPressingStow() {
    return this.getRawButton(ButtonMap.STOW_BUTTON);
  }

  public boolean isPressingRecive(){
    return this.getRawButton(ButtonMap.RECEIVE_BUTTON);
  }
  public boolean isPressingL1(){
    return this.getRawButton(ButtonMap.L1_BUTTON);
  }

  public boolean isPressingL2(){
    return this.getRawButton(ButtonMap.L2_BUTTON);
  }

  public boolean isPressingL3(){
    return this.getRawButton(ButtonMap.L3_BUTTON);
  }

  public boolean isPressingL4(){
    return this.getRawButton(ButtonMap.L4_BUTTON);
  }

  public boolean isPressingOuttake(){
    return this.getRawButton(ButtonMap.OUTTAKE_BUTTON);
  }

  public boolean isPressingIntake(){
    return this.getRawButton(ButtonMap.INTAKE_BUTTON);
  }

  public boolean isPressingClimb(){
    return this.getRawButton(ButtonMap.CLIMB_BUTTON);
  }
  
  public boolean isPressingEngageAnchor(){
    return this.getRawButton(ButtonMap.ENGAGEANCHOR_BUTTON);
  }
}
