// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    kStow(1.2192),
    kLevel1(0),
    kLevel2(0),
    kLevel3(0),
    kLevel4(0),
    kCoralStation(0);

    /** 
     * meters
     */
    public final double height;

    ElevatorPosition(double height) {
      this.height = height;
    }
  }

  private final SparkMax leader;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorPID;

  public Elevator() {
    var elevatorEncoderConfig = new EncoderConfig()
        .positionConversionFactor(Constants.kElevatorEncoderPositionConversionFactor)
        .velocityConversionFactor(Constants.kElevatorEncoderVelocityConversionFactor);
    var elevatorClosedLoopConfig = new ClosedLoopConfig()
        .pid(Constants.kElevatorP, Constants.kElevatorI, Constants.kElevatorD, ClosedLoopSlot.kSlot0);
    var elevatorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(false)
        .apply(elevatorEncoderConfig)
        .apply(elevatorClosedLoopConfig);
    leader = new SparkMax(RobotMap.ELEVATOR_DRIVE_ID, MotorType.kBrushless);
    leader.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    elevatorPID = leader.getClosedLoopController();
    elevatorEncoder = leader.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHeight() {
    return elevatorEncoder.getPosition() + Constants.kRobotElevatorStowHeight;
  }

  public void setHeight(ElevatorPosition position) {
    double setpoint = position.height - Constants.kRobotElevatorStowHeight;
    elevatorPID.setReference(setpoint, ControlType.kPosition);
  }
}
