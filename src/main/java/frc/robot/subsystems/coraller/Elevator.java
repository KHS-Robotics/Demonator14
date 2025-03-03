package frc.robot.subsystems.coraller;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;
import frc.robot.subsystems.coraller.CorallerSetpoints.ElevatorSetpoints;

class Elevator extends SubsystemBase {
  private final SparkMax leader, follower;
  private final RelativeEncoder encoder;
  private final SparkAnalogSensor absEncoder;
  private final SparkLimitSwitch bottomLimitSwitch;
  private final ElevatorPID pid;
  // TODO: Absolute encoder / potentiometer for position?

  /** The current setpoint measured from the ground. */
  private double setpointHeightFromGroundInches;

  public Elevator() {
    super(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName());

    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(ElevatorConfig.kElevatorEncoderPositionConversionFactor)
        .velocityConversionFactor(ElevatorConfig.kElevatorEncoderVelocityConversionFactor);

    var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyClosed);

    var leaderConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false)
        .apply(relativeEncoderConfig)
        .apply(limitSwitchConfig);
    leader = new SparkMax(RobotMap.ELEVATOR_DRIVE_LEADER_ID, MotorType.kBrushless);
    leader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .follow(RobotMap.ELEVATOR_DRIVE_LEADER_ID, true)
        .apply(relativeEncoderConfig);
    follower = new SparkMax(RobotMap.ELEVATOR_DRIVE_FOLLOWER_ID, MotorType.kBrushless);
    follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = leader.getEncoder();
    encoder.setPosition(0);
    
    absEncoder = leader.getAnalog();

    bottomLimitSwitch = leader.getReverseLimitSwitch();

    if (ElevatorConfig.kElevatorPIDMode == ElevatorConfig.PIDMode.TRAPEZOID){
      pid = new ElevatorPIDTrapazoid();
    } else {
      pid = new ElevatorPIDDefault();
    }

    SmartDashboard.putData(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName(), this);
    SmartDashboard.putData(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName() + "/PID", pid);

    setpointHeightFromGroundInches = ElevatorSetpoints.STOW_HEIGHT;
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopElevator");
  }

  public Command setHeightCommand(double heightFromGround) {
    var cmd = this.run(() -> setSetpointHeight(heightFromGround)).until(this::isAtSetpoint);
    return cmd.withName("SetElevatorSetpoint");
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
  public void setSetpointHeight(double heightFromGroundInches) {
    // extra precaution to prevent too low of setpoints
    if (heightFromGroundInches < ElevatorSetpoints.STOW_HEIGHT) {
      heightFromGroundInches = ElevatorSetpoints.STOW_HEIGHT;
    }
    // extra precaution to prevent too large of setpoints
    if (heightFromGroundInches > ElevatorSetpoints.L4_HEIGHT) {
      heightFromGroundInches = ElevatorSetpoints.L4_HEIGHT;
    }

    // only reset for new setpoints
    if (setpointHeightFromGroundInches != heightFromGroundInches) {
      pid.reset(getHeightFromGroundInches());
    }
    setpointHeightFromGroundInches = heightFromGroundInches;
  }

  /** Updates the setpoint to the current position. */
  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointHeight(getHeightFromGroundInches());
      // reset RelativeEncoder when sitting at bottom in disabled
      if (isAtBottom()) {
        encoder.setPosition(0);
      }
    }
  }

  public boolean isAtBottom() {
    return bottomLimitSwitch.isPressed();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGroundInches - getHeightFromGroundInches());
    return (error < 0.25);
  }

  public double getHeightFromGroundInches() {
    return encoder.getPosition() + ElevatorSetpoints.STOW_HEIGHT;
  }

  private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getHeightFromGroundInches(), setpointHeightFromGroundInches);
    var output = pidOutput + ElevatorConfig.kElevatorKG;

    // prevent trying to move past the bottom or setting motor outputs while limit
    // switch is pressed when the setpoint is the stow height
    if ((output < 0 || setpointHeightFromGroundInches == ElevatorSetpoints.STOW_HEIGHT) && isAtBottom()) {
      output = 0;
    }

    // reset elevator when stowed and reaches the bottom
    if (setpointHeightFromGroundInches == ElevatorSetpoints.STOW_HEIGHT && isAtBottom()) {
      encoder.setPosition(0);
    }

    // limit down voltage
    output = MathUtil.clamp(output, -5, 12);

    leader.setVoltage(output);
  }

  private double getVoltage() {
    return absEncoder.getPosition();
  }

  public void stop() {
    leader.stopMotor();
    pid.reset(getHeightFromGroundInches());
    setSetpointHeight(getHeightFromGroundInches());
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
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("IsAtBottom", this::isAtBottom, null);
    builder.addDoubleProperty("PotVoltage", this::getVoltage, null);
  }
}
