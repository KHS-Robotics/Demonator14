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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.CorallerConfig;

class Elevator extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;

  private double setpointHeightFromGround = CorallerConfig.kRobotElevatorStowHeightInches;
  private double setpointHeightFromElevatorBottom;

  public Elevator() {
    super("Coraller/Elevator");
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

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName()+"/PID Controller", pid);
  }

  @Override
  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("StopElevator");
  }

  public Command setSetpointCommand(double heightFromGround) {
    return this.run(() -> setSetpoint(heightFromGround)).until(this::isAtSetpoint).withName("SetElevatorSetpoint");
  }

  /** 
   * This method sets the configured setpoint for the elevator. The periodic function
   * is contantly polling this value to make adjustments when it changes
   */
  public void setSetpoint(double heightFromGround) {
    // extra precaution to prevent negative setpoints
    if (heightFromGround < CorallerConfig.kRobotElevatorStowHeightInches) {
      heightFromGround = CorallerConfig.kRobotElevatorStowHeightInches;
      DriverStation.reportWarning("Attempting to set an Elevator height lower than the stow height!", false);
    }

    setpointHeightFromGround = heightFromGround;
    setpointHeightFromElevatorBottom = heightFromGround - CorallerConfig.kRobotElevatorStowHeightInches;
  }

  public boolean isAtBottom() {
    // TODO: use limit switch / talon tach!
    return true;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointHeightFromGround - getHeightFromGroundInches());
    return (error < 1);
  }

  public double getHeightFromElevatorBottomInches() {
    return encoder.getPosition();
  }

  private double getHeightFromGroundInches() {
    return encoder.getPosition() + CorallerConfig.kRobotElevatorStowHeightInches;
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var output = pid.calculate(getHeightFromElevatorBottomInches(), setpointHeightFromElevatorBottom);
    motor.setVoltage(output);
  }

  public void stop(){
    motor.stopMotor();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("SetPointFromGround", () -> setpointHeightFromGround, this::setSetpoint);
    builder.addDoubleProperty("HeightFromGround", this::getHeightFromGroundInches,null);
    builder.addDoubleProperty("HeightFromBottom", this::getHeightFromElevatorBottomInches, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
    builder.addBooleanProperty("IsAtBottom", this::isAtBottom, null);
  }
}
