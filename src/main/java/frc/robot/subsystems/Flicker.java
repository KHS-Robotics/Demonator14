// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlickerConfig;
import frc.robot.RobotMap;

public class Flicker extends SubsystemBase {
  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final PIDController pid;
  private double setpointAngleDegrees;


  /** Creates a new Flicker. */
  public Flicker() {
    var flickerConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false);

    motor = new SparkMax(RobotMap.ALGAE_FLICKER_MOTOR_ID, MotorType.kBrushless);
    motor.configure(flickerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    
    encoder = motor.getAbsoluteEncoder();

    pid = new PIDController(FlickerConfig.kFlickerP, FlickerConfig.kFlickerI, FlickerConfig.kFlickerD);
    pid.setIZone(5);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

  }


  public Command setAngleCommand(double angleDegrees) {
    return this.run(() -> setSetpointAngle(angleDegrees))
      .until(this::isAtSetpoint)
      .withName("SetAnglerSetpoint");
  }

public void setSetpointAngle(double setpointDegrees) {
  // only reset for new setpoints
  if (setpointDegrees != setpointAngleDegrees) {
    pid.reset();
  }
  setpointAngleDegrees = setpointDegrees;
}

public double getAngle() {
  return encoder.getPosition();
}

public boolean isAtSetpoint() {
  var error = Math.abs(setpointAngleDegrees - getAngle());
  return (error < 1);
}


public void stop() {
  motor.stopMotor();
  pid.reset();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
