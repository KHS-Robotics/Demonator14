// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  public enum AnchorPosition {
    kEngaged(0.5),
    kUnengaged(0.0);

    public final double percent;

    AnchorPosition(double percent) {
      this.percent = percent;
    }
  }

  private AnchorPosition currentPosition = AnchorPosition.kUnengaged;

  private final SparkMax reel;
  private final Servo anchor;
  private boolean reelingIn;
  private boolean reelingOut;

  public Climber() {
    var reelConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false);
    reel = new SparkMax(RobotMap.CLIMBER_REEL_ID, MotorType.kBrushless);
    reel.configure(reelConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    anchor = new Servo(RobotMap.CLIMBER_ANCHOR_ID);
    SmartDashboard.putData(getName(), this);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber-Engaged", isEngaged());
  }

  // TODO() full speed once we know direction
  public void reelIn() {
    reel.setVoltage(6);
    reelingIn = true;
    reelingOut = false;
  }

  public void reelOut() {
    reel.setVoltage(-6);
    reelingIn = false;
    reelingOut = true;
  }

  public void reelStop(){
    reel.stopMotor();
    reelingIn = false;
    reelingOut = false;
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

  private String reelingStatus(){
    if (!reelingIn && !reelingOut){
      return "Not reeling";
    }else if (reelingIn){
      return "reeling in";
    }else if (reelingOut){
      return "reeling out";
    }else{
      return "I don't know what is happening";
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::reelStop);
    builder.setActuator(true);
    builder.addBooleanProperty("IsAnchorEngaged", this::isEngaged, null);
    builder.addStringProperty("ReelingStatus", this::reelingStatus, null);
  }
}
