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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private AnchorState currentPosition = AnchorState.kUnengaged;

  private final SparkMax reel;
  private final Servo anchor;
  private ReelState reelState;

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
    setAnchorPosition(AnchorState.kUnengaged);

    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  public Command engageAnchor() {
    return setAnchor(AnchorState.kEngaged);
  }

  public Command unengageAnchor() {
    return setAnchor(AnchorState.kUnengaged);
  }

  private Command setAnchor(AnchorState position) {
    var cmd = run(() -> setAnchor(position)).withTimeout(0.5);
    return cmd.withName("SetAnchor(\"" + position.toString() + "\")");
  }

  private void setAnchorPosition(AnchorState position) {
    currentPosition = position;
    anchor.set(position.percent);
  }

  public boolean isEngaged() {
    return currentPosition == AnchorState.kEngaged;
  }

  public Command reelIn() {
    // TODO: full speed once we know direction???
    var cmd = startEnd(() -> setReel(6), this::stop);
    return cmd.withName("ClimberReelIn");
  }

  public Command reelOut() {
    // TODO: full speed once we know direction???
    var cmd = startEnd(() -> setReel(-6), this::stop);
    return cmd.withName("ClimberReelOut");
  }

  private void setReel(double voltage) {
    reelState = voltage > 0 ? ReelState.REELING_IN : voltage < 0 ? ReelState.REELING_OUT : ReelState.OFF;
    reel.setVoltage(voltage);
  }

  public void stop() {
    setReel(0);
  }

  /**
   * Positions for the climber anchor controlled by a servo.
   * 
   * @see {@link edu.wpi.first.wpilibj.Servo#set(double)}
   */
  private enum AnchorState {
    kEngaged(0.5),
    kUnengaged(0.0);

    /**
     * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     */
    public final double percent;

    private AnchorState(double percent) {
      this.percent = percent;
    }
  }
  
  private enum ReelState {
    OFF("Off"),
    REELING_IN("Reeling In"),
    REELING_OUT("Reeling Out");

    private final String state;

    private ReelState(String s) {
      state = s;
    }

    public String toString() {
      return this.state;
    }
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addBooleanProperty("IsAnchorEngaged", this::isEngaged, null);
    builder.addStringProperty("ReelingStatus", reelState::toString, null);
  }
}
