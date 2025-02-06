package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.ElevatorConfig;
import frc.robot.subsystems.coraller.Coraller;

public class Elevator extends SubsystemBase {
  private final SparkMax leader, follower;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch bottomLimitSwitch;
  private final PIDController pid;

  private double setpointHeightFromGroundInches = ElevatorConfig.STOW_HEIGHT;
  private double setpointHeightFromBottomInches;

  public Elevator() {
    super(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName());

    var elevatorEncoderConfig = new EncoderConfig()
      .positionConversionFactor(ElevatorConfig.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(ElevatorConfig.kElevatorEncoderVelocityConversionFactor);

    var leaderConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(45)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false)
      .apply(elevatorEncoderConfig);
    leader = new SparkMax(RobotMap.ELEVATOR_DRIVE_LEADER_ID, MotorType.kBrushless);
    leader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(45)
      .follow(RobotMap.ELEVATOR_DRIVE_LEADER_ID, true)
      .apply(elevatorEncoderConfig);
    follower = new SparkMax(RobotMap.ELEVATOR_DRIVE_FOLLOWER_ID, MotorType.kBrushless);
    follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = leader.getEncoder();
    // TODO: should be reverse? something to check...
    bottomLimitSwitch = leader.getReverseLimitSwitch();

    pid = new PIDController(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD);
    pid.setIZone(3);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopElevator");
  }

  public Command setHeightCommand(double heightFromGround) {
    return this.run(() -> setSetpointHeight(heightFromGround))
      .until(this::isAtSetpoint)
      .withName("SetElevatorSetpoint");
  }

  public Command stow() {
    return setHeightCommand(setpointHeightFromGroundInches);
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
  public void setSetpointHeight(double heightFromGroundInches) {
    // extra precaution to prevent negative setpoints
    if (heightFromGroundInches < setpointHeightFromGroundInches) {
      heightFromGroundInches = setpointHeightFromGroundInches;
    }

    // only reset for new setpoints
    if (setpointHeightFromGroundInches != heightFromGroundInches) {
      pid.reset();
    }
    setpointHeightFromGroundInches = heightFromGroundInches;
    setpointHeightFromBottomInches = heightFromGroundInches - setpointHeightFromGroundInches;
  }

  /** Updates the setpoints to the current positions. */
  public void updateSetpointsForDisabledMode() {
    // only in disabled
    if (RobotState.isDisabled()) {
      // angler
      var isElevatorEncoderNonNegative = getHeightFromBottomInches() >= 0;
      setSetpointHeight(isElevatorEncoderNonNegative ? getHeightFromGroundInches() : setpointHeightFromGroundInches);
    }
  }

  public boolean isAtBottom() {
    return bottomLimitSwitch.isPressed();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGroundInches - getHeightFromGroundInches());
    return (error < 1);
  }

  public double getHeightFromBottomInches() {
    return encoder.getPosition();
  }

  public double getHeightFromGroundInches() {
    return encoder.getPosition() + setpointHeightFromGroundInches;
  }

  public void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getHeightFromBottomInches(), setpointHeightFromBottomInches);
    var output = pidOutput + ElevatorConfig.kElevatorKG;

    // prevent trying to move past the bottom or setting motor outputs while limit
    // switch is pressed when the setpoint is the stow height
    if ((output < 0 || setpointHeightFromGroundInches == setpointHeightFromGroundInches) && isAtBottom()) {
      output = 0;
    }

    leader.setVoltage(output);
  }

  public void stop() {
    leader.stopMotor();
    pid.reset();
  }
  
  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("SetPointFromGround", () -> setpointHeightFromGroundInches, this::setSetpointHeight);
    builder.addDoubleProperty("HeightFromGround", this::getHeightFromGroundInches, null);
    builder.addDoubleProperty("HeightFromBottom", this::getHeightFromBottomInches, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("IsAtBottom", this::isAtBottom, null);
  }
}
