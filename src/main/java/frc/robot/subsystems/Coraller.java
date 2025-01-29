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
import frc.robot.Constants.CorallerConfig;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ElevatorConfig;



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

  
  public enum ElevatorPosition {
    kStow(48),
    kLevel1(0),
    kLevel2(0),
    kLevel3(0),
    kLevel4(72),
    kCoralStation(0);

    /**
     * inches
     */
    public final double height;

    ElevatorPosition(double height) {
      this.height = height;
    }
  }

  private ElevatorPosition elevatorSetPoint = ElevatorPosition.kStow;

  private final SparkMax leader;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorPID;


  private CorallerPosition corallerSetPoint= CorallerPosition.kStow;
  private final SparkMax angler;
  private final AbsoluteEncoder angleEncoder;
  private final SparkClosedLoopController corallerPID;



  /**
   * intake/outtake
   */
  private final SparkMax spitter;

  public Coraller() {
    var elevatorEncoderConfig = new EncoderConfig()
        .positionConversionFactor(ElevatorConfig.kElevatorEncoderPositionConversionFactor)
        .velocityConversionFactor(ElevatorConfig.kElevatorEncoderVelocityConversionFactor);
    var elevatorClosedLoopConfig = new ClosedLoopConfig()
        .pid(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD, ClosedLoopSlot.kSlot0);
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

  public double getHeightFromGround() {
    return elevatorEncoder.getPosition() + ElevatorConfig.kRobotElevatorStowHeightInches;
  }

    // sets height relative to the floor
  public void setHeightPosition(ElevatorPosition position) {
    elevatorSetPoint = position;
    double setpoint = position.height - ElevatorConfig.kRobotElevatorStowHeightInches;
    elevatorSetPosition(setpoint);
  }


  // set height relative to bottom of elevator
  public void elevatorSetPosition (double setpoint) {
    elevatorPID.setReference(setpoint, ControlType.kPosition);
  }

  
  // TODO() add once we know what sensor we are using
  public boolean isElevatorAtBottom() {
    return true;
  }
  
  public ElevatorPosition getElevatorSetPoint(){
    return elevatorSetPoint;
  }

  public boolean elevatorIsAtSetPoint(){
    var error = Math.abs(elevatorSetPoint.height - getHeightFromGround());
    return (error < 1);
  }

    // sets height relative to the floor
  public void setElevatorPosition(ElevatorPosition position) {
    elevatorSetPoint = position;
    double setpoint = position.height - ElevatorConfig.kRobotElevatorStowHeightInches;
    elevatorSetPosition(setpoint);
  }

    // get height relative to bottom of elevator
  public double getRelativePosition() {
    return elevatorEncoder.getPosition();
 }
    //end of elevator functions
  // TODO() find out volts and which are inversed

  public void spitterIn() {
    spitter.setVoltage(6);
  }

  public void spitterOut() {
    spitter.setVoltage(-6);
  }

  public double getCorallerAngle(){
    return angleEncoder.getPosition();
  }

  public void setCorallerPosition(CorallerPosition pos) {
   corallerSetPoint = pos;
    setCorallerSetPoint (pos.degrees);
  }

  public void setCorallerSetPoint (double setpoint) {
    corallerPID.setReference(setpoint, ControlType.kPosition);
  }

  public CorallerPosition getCorallerPosition() {
    return corallerSetPoint;
  }

  public boolean isAtCorallerSetPoint () {
    var error = Math.abs(corallerSetPoint.degrees - getCorallerAngle ());
    return (error < 1);
  }
}
