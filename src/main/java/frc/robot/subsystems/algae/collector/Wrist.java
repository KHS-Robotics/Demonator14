// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae.collector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.subsystems.algae.collector.AlgaeCollectorConfig.AlgaeWristConfig;
import frc.robot.subsystems.algae.collector.AlgaeCollectorSetpoints.WristSetpoints;

class Wrist extends SubsystemBase {
  private double setpointAngleDegrees;
  // for setpoint debouncing
  private boolean hasReachedSetpoint = false;

  private final SparkMax motor;
  private final AbsoluteEncoder encoder;

  public Wrist() {
    super(AlgaeCollector.class.getSimpleName() + "/" + Wrist.class.getSimpleName());

    var encoderConfig = new AbsoluteEncoderConfig()
      .inverted(true);
    var algaeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(30)
      .inverted(true)
      .apply(encoderConfig);
    motor = new SparkMax(RobotMap.ALGAE_WRIST_MOTOR_ID, MotorType.kBrushless);
    motor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    SmartDashboard.putData(getName(), this);
  }

  /** {@inheritDoc} */
  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

  public Command deploy() {
    hasReachedSetpoint = false;
    setSetpointAngle(WristSetpoints.DEPLOY);
    return this.run(() -> setSetpointAngle(WristSetpoints.DEPLOY))
        .until(() -> hasReachedSetpoint)
        .withName("DeployWrist");
  }

  public Command stow() {
    hasReachedSetpoint = false;
    setSetpointAngle(WristSetpoints.STOW);
    return this.run(() -> setSetpointAngle(WristSetpoints.STOW))
        .until(() -> hasReachedSetpoint)
        .withName("StowWrist");
  }

  public Command stopCommand() {
    return runOnce(this::stop)
        .withName("StopWrist");
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
  public void setSetpointAngle(double setpointDegrees) {
    setpointAngleDegrees = setpointDegrees;
  }

  public double getAngle() {
    // 0 is stowed
    return Units.rotationsToDegrees(encoder.getPosition()) + AlgaeWristConfig.kAlgaeWristOffset;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 5);
  }

  private void setMotorOutputForSetpoint() {
    // bang bang control since there are only two setpoints
    // stowed + deployed
    hasReachedSetpoint |= isAtSetpoint();
    var error = !hasReachedSetpoint ? setpointAngleDegrees - getAngle() : 0;
    var output = 2 * Math.signum(error);
    motor.setVoltage(output);
  }

  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointAngle(getAngle());
      hasReachedSetpoint = true;
    }
  }

  public void stop() {
    motor.stopMotor();
    hasReachedSetpoint = true;
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("HasReached", () -> hasReachedSetpoint, null);
    builder.addDoubleProperty("SetpointAngle", () -> setpointAngleDegrees, null);
  }
}
