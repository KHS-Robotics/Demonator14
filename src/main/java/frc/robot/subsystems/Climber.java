// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private AnchorPosition currentPosition = AnchorPosition.kUnengaged;

  private final SparkMax reel;
  private final Servo anchor;

  public Climber() {
    var reelConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      // TODO: invert for our desired sign (positive to reelIn)
      .inverted(false);
    reel = new SparkMax(RobotMap.CLIMBER_REEL_ID, MotorType.kBrushless);
    reel.configure(reelConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    anchor = new Servo(RobotMap.CLIMBER_ANCHOR_ID);
    setAnchorPosition(AnchorPosition.kUnengaged);

    SmartDashboard.putData(this);
  }

  public void reelIn() {
    // TODO: full speed once we know direction
    reel.setVoltage(6);
  }

  public void reelOut() {
    // TODO: full speed??? once we know direction
    reel.setVoltage(-6);
  }

  public void setAnchorPosition(AnchorPosition position) {
    currentPosition = position;
    anchor.set(position.percent);
  }

  public boolean isEngaged() {
    return getAnchorPosition() == AnchorPosition.kEngaged;
  }

  public AnchorPosition getAnchorPosition() {
    return currentPosition;
  }

  /**
   * Positions for the climber anchor controlled by a servo.
   * 
   * @see {@link edu.wpi.first.wpilibj.Servo#set(double)}
   */
  public enum AnchorPosition {
    kEngaged(0.5),
    kUnengaged(0.0);

    /**
     * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     */
    public final double percent;

    private AnchorPosition(double percent) {
      this.percent = percent;
    }
  }
}
