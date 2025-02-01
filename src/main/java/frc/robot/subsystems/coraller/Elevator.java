package frc.robot.subsystems.coraller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.CorallerConfig;

class Elevator extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double setPointHeight;

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
    pid = motor.getClosedLoopController();
    encoder = motor.getEncoder();
  }

  


  public Command setPosition(double position) {
    return this.runOnce(() -> setPositionInternal(position));
  }

    // sets height relative to the floor
  private void setPositionInternal(double position) {
    setPointHeight = position;
    double setpoint = position - CorallerConfig.kRobotElevatorStowHeightInches;
    changeSetPoint(setpoint);
  }

  private void changeSetPoint(double setpoint) {
    pid.setReference(setpoint, ControlType.kPosition);
  }

  public boolean isElevatorAtBottom() {
    return getHeightFromGround() == CorallerConfig.kRobotElevatorStowHeightInches;
  }

  public double getSetPoint() {
    return setPointHeight;
  }

  public boolean isAtSetPoint() {
    var error = Math.abs(setPointHeight - getHeightFromGround());
    return (error < 1);
  }

  // get height relative to bottom of elevator
  public double getRelativePosition() {
    return encoder.getPosition();
  }

  private double getHeightFromGround() {
    return encoder.getPosition() + CorallerConfig.kRobotElevatorStowHeightInches;
  }
}
