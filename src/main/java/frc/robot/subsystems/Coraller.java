// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CorallerConfig;
import frc.robot.RobotMap;

public class Coraller extends SubsystemBase {
  public enum CorallerPosition {
    kStow(0),
    kLevel1(0),
    kLevel2(0),
    kLevel3(0),
    kLevel4(0),
    kCoralStation(0);

    /**
     * degrees
     */
    public final double degrees;

    CorallerPosition(double degrees) {
      this.degrees = degrees;
    }
  }

  private CorallerPosition currentSetpoint = CorallerPosition.kStow;
  private final SparkMax angler;
  private final AbsoluteEncoder angleEncoder;
  private final SparkClosedLoopController corallerPID;
  /**
   * intake/outtake
   */
  private final SparkMax spitter;

  public Coraller() {
    var corallerClosedLoopConfig = new ClosedLoopConfig()
        .pid(CorallerConfig.kCorallerP, CorallerConfig.kCorallerI, CorallerConfig.kCorallerD,
            ClosedLoopSlot.kSlot0);
    var corallerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false)
        .apply(corallerClosedLoopConfig);
    angler = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    angler.configure(corallerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    angleEncoder = angler.getAbsoluteEncoder();
    corallerPID = angler.getClosedLoopController();

    var spitterConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
    spitter = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    spitter.configure(spitterConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAngle() {
    return angleEncoder.getPosition();
  }

  public void setAngle(CorallerPosition pos) {
    currentSetpoint = pos;
    corallerPID.setReference(pos.degrees, ControlType.kPosition);
  }

  public CorallerPosition getSetpoint() {
    return currentSetpoint;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(currentSetpoint.degrees - getAngle());
    return (error < 1);
  }
}
