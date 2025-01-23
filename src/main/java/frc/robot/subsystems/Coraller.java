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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Coraller extends SubsystemBase {
  private final SparkMax angler;
  private final RelativeEncoder corallerEncoder;
  private final SparkClosedLoopController corallerPID;
  /**
   * intake/outtake
   */
  private final SparkMax spitter;
  
  public Coraller() {
    
    var corallerEncoderConfig = new EncoderConfig()
        .positionConversionFactor(Constants.kCorallerEncoderPositionConversionFactor)
        .velocityConversionFactor(Constants.kCorallerEncoderVelocityConversionFactor);
    var corallerClosedLoopConfig = new ClosedLoopConfig()
        .pid(Constants.kCorallerP, Constants.kCorallerI, Constants.kCorallerD, ClosedLoopSlot.kSlot0);
    var corallerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(false)
        .apply(corallerEncoderConfig)
        .apply(corallerClosedLoopConfig);
    angler = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    angler.configure(corallerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    corallerPID = angler.getClosedLoopController();
    corallerEncoder = angler.getEncoder();

    var spitterConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(false);
    spitter = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    spitter.configure(spitterConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
