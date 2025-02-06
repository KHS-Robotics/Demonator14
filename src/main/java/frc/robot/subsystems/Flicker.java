// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlickerConfig;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Flicker extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;

  private final Elevator elevator = RobotContainer.kElevator;
  private double setpointAngleDegrees;

  /** Creates a new Flicker. */
  public Flicker() {
    var flickerEncoderConfig = new EncoderConfig()
        .positionConversionFactor(FlickerConfig.kFlickerEncoderPositionConversionFactor)
        .velocityConversionFactor(FlickerConfig.kFlickerEncoderVelocityConversionFactor);
    var flickerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        // TODO: set inverted based on our desired sign of direction (positive up /
        // negative down)
        .inverted(false)
        .apply(flickerEncoderConfig);
    motor = new SparkMax(RobotMap.ALGAE_FLICKER_MOTOR_ID, MotorType.kBrushless);
    motor.configure(flickerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getEncoder();

    pid = new PIDController(FlickerConfig.kFlickerP, FlickerConfig.kFlickerI, FlickerConfig.kFlickerD);
    pid.setIZone(5);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

  }

  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command prepareToFlickCommand(ReefScoringConfiguration cfg) {
    var cmd = Commands.parallel(elevator.setHeightCommand(cfg.elevatorPosition), setAngleCommand(cfg.flickerPosition));
    return cmd.withName("PrepareToFlick");
  }

  public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetAnglerSetpoint");
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

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);
    var ffGravity = FlickerConfig.kFlickerKG * Math.cos(getAngle());
    var output = pidOutput + ffGravity;
    motor.setVoltage(output);
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopFlicker");
  }

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

  /** Heights and angles to score on the reef. */
  public enum ReefScoringConfiguration {
    STOW(FlickerConfig.STOW_HEIGHT, FlickerConfig.STOW_ANGLE),
    L2(FlickerConfig.L2_HEIGHT, FlickerConfig.L2_ANGLE),
    L3(FlickerConfig.L3_HEIGHT, FlickerConfig.L3_ANGLE);

    /** Inches */
    private final double elevatorPosition;
    /** Degrees */
    private final double flickerPosition;

    private ReefScoringConfiguration(double elevatorPosition, double anglerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.flickerPosition = anglerPosition;
    }
  }
}
