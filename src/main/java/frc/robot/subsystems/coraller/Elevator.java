package frc.robot.subsystems.coraller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.CorallerConfig;

class Elevator extends SubsystemBase {
  private final SparkMax leader, follower;
  private final RelativeEncoder encoder;
  private final PIDController pid;

  private double setpointHeightFromGroundInches = CorallerConfig.STOW_HEIGHT;
  private double setpointHeightFromElevatorBottomInches;

  public Elevator() {
    super(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName());

    var elevatorEncoderConfig = new EncoderConfig()
      .positionConversionFactor(CorallerConfig.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(CorallerConfig.kElevatorEncoderVelocityConversionFactor);

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

    pid = new PIDController(CorallerConfig.kElevatorP, CorallerConfig.kElevatorI, CorallerConfig.kElevatorD);
    pid.setIZone(3);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopElevator");
  }

  public Command setSetpointCommand(double heightFromGround) {
    return this.run(() -> setSetpoint(heightFromGround))
      .until(this::isAtSetpoint)
      .withName("SetElevatorSetpoint");
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
  public void setSetpoint(double heightFromGroundInches) {
    // extra precaution to prevent negative setpoints
    if (heightFromGroundInches < CorallerConfig.STOW_HEIGHT) {
      heightFromGroundInches = CorallerConfig.STOW_HEIGHT;
    }

    // only reset for new setpoints
    if (setpointHeightFromGroundInches != heightFromGroundInches) {
      pid.reset();
    }
    setpointHeightFromGroundInches = heightFromGroundInches;
    setpointHeightFromElevatorBottomInches = heightFromGroundInches - CorallerConfig.STOW_HEIGHT;
  }

  public boolean isAtBottom() {
    // TODO: use limit switch / talon tach!
    return true;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGroundInches - getHeightFromGroundInches());
    return (error < 1);
  }

  public double getHeightFromElevatorBottomInches() {
    return encoder.getPosition();
  }

  public double getHeightFromGroundInches() {
    return encoder.getPosition() + CorallerConfig.STOW_HEIGHT;
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getHeightFromElevatorBottomInches(), setpointHeightFromElevatorBottomInches);
    var output = pidOutput + CorallerConfig.kElevatorKG;

    // prevent trying to move past the bottom or setting motor outputs while limit
    // switch is pressed when the setpoint is the stow height
    if ((output < 0 || setpointHeightFromGroundInches == CorallerConfig.STOW_HEIGHT) && isAtBottom()) {
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
    builder.addDoubleProperty("SetPointFromGround", () -> setpointHeightFromGroundInches, this::setSetpoint);
    builder.addDoubleProperty("HeightFromGround", this::getHeightFromGroundInches, null);
    builder.addDoubleProperty("HeightFromBottom", this::getHeightFromElevatorBottomInches, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("IsAtBottom", this::isAtBottom, null);
  }
}
