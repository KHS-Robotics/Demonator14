// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae.collector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.AlgaeWristConfig;
import frc.robot.Constants.CorallerConfig;

class Wrist extends SubsystemBase {
  /** Creates a new AlgaeWrist. */
  private double setpointAngleDegrees;

  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final PIDController pid;

  public Wrist() {
    var algaeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false);
    motor = new SparkMax(RobotMap.ALGAE_WRIST_MOTOR_ID, MotorType.kBrushless);
    motor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    pid = new PIDController(AlgaeWristConfig.kAlgaeP, AlgaeWristConfig.kAlgaeI, AlgaeWristConfig.kAlgaeD);
    pid.setIZone(5);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
  }

  /** {@inheritDoc} */
  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command setAngleCommand(double angleDegrees) {
    return this.run(() -> setSetpointAngle(angleDegrees))
      .until(this::isAtSetpoint)
      .withName("SetAlgaeSetpoint");
  }

  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopAlgae");
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
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

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);
    var ffGravity = CorallerConfig.kAnglerKG * Math.cos(getAngle());
    var output = pidOutput + ffGravity;
    motor.setVoltage(output);
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Setpoint", () -> setpointAngleDegrees, this::setSetpointAngle);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
  }
  }
