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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private final SparkMax reel;
  private final Servo anchor;
  public Climber() {
    var reelConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
    reel = new SparkMax(RobotMap.CLIMBER_REEL_ID, MotorType.kBrushless);
    reel.configure(reelConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    anchor = new Servo(RobotMap.CLIMBER_ANCHOR_ID);
    //does the servo need a config?
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
