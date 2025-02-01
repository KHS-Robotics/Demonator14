package frc.robot.subsystems.coraller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.CorallerConfig;

class Elevator extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;

  private double setpointHeightFromGround;
  private double setpointHeightRelative;

  public Elevator() {
    var elevatorEncoderConfig = new EncoderConfig()
        .positionConversionFactor(CorallerConfig.kElevatorEncoderPositionConversionFactor)
        .velocityConversionFactor(CorallerConfig.kElevatorEncoderVelocityConversionFactor);
    var elevatorClosedLoopConfig = new ClosedLoopConfig()
        .pid(CorallerConfig.kElevatorP, CorallerConfig.kElevatorI, CorallerConfig.kElevatorD, ClosedLoopSlot.kSlot0);
    var elevatorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(false)
        .apply(elevatorEncoderConfig)
        .apply(elevatorClosedLoopConfig);
    motor = new SparkMax(RobotMap.ELEVATOR_DRIVE_ID, MotorType.kBrushless);
    motor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    pid = new PIDController(CorallerConfig.kElevatorP, CorallerConfig.kElevatorI, CorallerConfig.kElevatorD);
    encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command setSetpointComnand(double heightFromGround) {
    return this.run(() -> setSetoint(heightFromGround)).until(this::isAtSetpoint);
  }

  public void setSetoint(double heightFromGround) {
    setpointHeightFromGround = heightFromGround;
    setpointHeightRelative = heightFromGround - CorallerConfig.kRobotElevatorStowHeightInches;
  }

  public boolean isAtBottom() {
    // TODO: use limit switch / talon tach!
    return true;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGround - getHeightFromGround());
    return (error < 1);
  }

  public double getRelativePosition() {
    return encoder.getPosition();
  }

  private double getHeightFromGround() {
    return encoder.getPosition() + CorallerConfig.kRobotElevatorStowHeightInches;
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var output = pid.calculate(getRelativePosition(), setpointHeightRelative);
    motor.setVoltage(output);
  }
}
