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
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.subsystems.coraller.CorallerConfig.ElevatorConfig;
import frc.robot.subsystems.coraller.CorallerSetpoints.ElevatorSetpoints;

class Elevator extends SubsystemBase {
  private final ElevatorHeightMode kHeightMode = ElevatorHeightMode.kRelative;
  private enum ElevatorHeightMode {
    kRelative, kAbsolute;
  }

  private final SparkMax leader, follower;
  private final RelativeEncoder relativeEncoder;
  private final SparkAnalogSensor absoluteEncoder;
  private final SparkLimitSwitch bottomLimitSwitch;
  private final PIDController pid;

  /** The current setpoint measured from the ground. */
  private double setpointHeightFromGroundInches = ElevatorSetpoints.STOW_HEIGHT;
  private SparkBaseConfig motorConfig = new SparkMaxConfig();
  private boolean overrideLimitSwitch = false;

  public Elevator() {
    super(Coraller.class.getSimpleName() + "/" + Elevator.class.getSimpleName());

    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(ElevatorConfig.kElevatorEncoderPositionConversionFactor)
        .velocityConversionFactor(ElevatorConfig.kElevatorEncoderVelocityConversionFactor);

    var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyClosed);

    motorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false)
        .apply(relativeEncoderConfig)
        .apply(limitSwitchConfig);
    leader = new SparkMax(RobotMap.ELEVATOR_DRIVE_LEADER_ID, MotorType.kBrushless);
    leader.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .follow(RobotMap.ELEVATOR_DRIVE_LEADER_ID, true)
        .apply(relativeEncoderConfig);
    follower = new SparkMax(RobotMap.ELEVATOR_DRIVE_FOLLOWER_ID, MotorType.kBrushless);
    follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    relativeEncoder = leader.getEncoder();
    
    absoluteEncoder = leader.getAnalog();

    bottomLimitSwitch = leader.getReverseLimitSwitch();

    pid = new PIDController(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD);
    pid.setIZone(3);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

    setSetpointHeight(getHeightFromGroundInches());
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

  public Command setOverride(boolean override) {
    var cmd = runOnce(() -> {
      overrideLimitSwitch = override;
  
      var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(!override)
        .reverseLimitSwitchType(Type.kNormallyClosed);
      motorConfig = motorConfig.apply(limitSwitchConfig);
      leader.configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
    return cmd.withName("SetElevatorOverride(\"" + override + "\")");
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
      pid.reset();
    }
    setpointHeightFromGroundInches = heightFromGroundInches;
  }

  /** Updates the setpoint to the current position. */
  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointHeight(getHeightFromGroundInches());
      // reset RelativeEncoder when sitting at bottom in disabled
      if (isAtBottomForRelativeEncoder()) {
        relativeEncoder.setPosition(0);
      }
    }
  }

  public boolean isAtBottomForRelativeEncoder() {
    return !overrideLimitSwitch && kHeightMode == ElevatorHeightMode.kRelative && bottomLimitSwitch.isPressed();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGroundInches - getHeightFromGroundInches());
    return (error < 0.25);
  }

  public double getHeightFromGroundInches() {
    switch(kHeightMode) {
      case kRelative:
        return getHeightFromGroundInchesUsingRelativeEncoder();
      case kAbsolute:
        return getHeightFromGroundInchesUsingAbsoluteEncoder();
      default:
        DriverStation.reportError("ELEVATOR: Invalid mode for measuring height.", false);
        pid.reset();
        return setpointHeightFromGroundInches;
    }
  }

  public double getHeightFromGroundInchesUsingRelativeEncoder() {
    var heightFromStow = relativeEncoder.getPosition();
    return heightFromStow + ElevatorSetpoints.STOW_HEIGHT;
  }

  public double getHeightFromGroundInchesUsingAbsoluteEncoder() {
    var deltaHeight = ElevatorSetpoints.L4_HEIGHT - ElevatorSetpoints.STOW_HEIGHT;
    var deltaVoltage = ElevatorConfig.kElevatorAbsoluteEncoderMaxVoltage - ElevatorConfig.kElevatorAbsoluteEncoderMinVoltage;
    var relativeVoltage = getPotentiometerVoltage() - ElevatorConfig.kElevatorAbsoluteEncoderMinVoltage;
    var heightFromStow = (deltaHeight * relativeVoltage) / deltaVoltage;
    return heightFromStow + ElevatorSetpoints.STOW_HEIGHT;
  }

  private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getHeightFromGroundInches(), setpointHeightFromGroundInches);
    var output = pidOutput + ElevatorConfig.kElevatorKG;

    // prevent trying to move past the bottom or setting motor outputs while limit
    // switch is pressed when the setpoint is the stow height
    if ((output < 0 || setpointHeightFromGroundInches == ElevatorSetpoints.STOW_HEIGHT) && isAtBottomForRelativeEncoder()) {
      output = 0;
    }

    // reset elevator when stowed and reaches the bottom
    if (setpointHeightFromGroundInches == ElevatorSetpoints.STOW_HEIGHT && isAtSetpoint() && isAtBottomForRelativeEncoder()) {
      relativeEncoder.setPosition(0);
    }

    // limit down voltage
    output = MathUtil.clamp(output, -5, 12);

    leader.setVoltage(output);
  }

  private double getPotentiometerVoltage() {
    return absoluteEncoder.getVoltage();
  }

  public void stop() {
    leader.stopMotor();
    pid.reset();
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
    builder.addDoubleProperty("RelativeEncoderHeightFromGround", this::getHeightFromGroundInchesUsingRelativeEncoder, null);
    builder.addDoubleProperty("AbsoluteEncoderHeightFromGround", this::getHeightFromGroundInchesUsingAbsoluteEncoder, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("IsAtBottomForRelativeEncoder", this::isAtBottomForRelativeEncoder, null);
    builder.addDoubleProperty("PotVoltage", this::getPotentiometerVoltage, null);
    builder.addStringProperty("ElevatorHeightMode", () -> kHeightMode.toString(), null);
    builder.addBooleanProperty("Override", () -> overrideLimitSwitch, null);
    builder.addBooleanProperty("TalonTach", () -> bottomLimitSwitch.isPressed(), null);
  }
}
