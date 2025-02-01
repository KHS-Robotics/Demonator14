package frc.robot.subsystems.coraller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorallerConfig;
import frc.robot.RobotMap;

public class Angler extends SubsystemBase {
  private double setpointAngle;

  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final PIDController pid;

  public Angler() {
    var anglerClosedLoopConfig = new ClosedLoopConfig()
        .pid(CorallerConfig.kAnglerP, CorallerConfig.kAnglerI, CorallerConfig.kAnglerD,
            ClosedLoopSlot.kSlot0);
    var anglerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false)
        .apply(anglerClosedLoopConfig);
    motor = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    motor.configure(anglerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    pid = new PIDController(CorallerConfig.kAnglerP, CorallerConfig.kAnglerI, CorallerConfig.kAnglerD);
    encoder = motor.getAbsoluteEncoder();
  }

  public void periodic() {
    setMotorOutputForSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setActuator(true);
    builder.addDoubleProperty("SetPoint", this::getSetPoint, this::setSetpoint);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtAnglerSetpoint, null);
  }


  public double getAngle() {
    return encoder.getPosition();
  }

  public Command setSetpointComnand(double angleDegrees) {
    return this.run(() -> setSetpoint(angleDegrees)).until(this::isAtAnglerSetpoint);
  }

  public void setSetpoint(double setpoint) {
    setpointAngle = setpoint;
  }

  public double getSetPoint() {
    return setpointAngle;
  }

  public boolean isAtAnglerSetpoint() {
    var error = Math.abs(setpointAngle - getAngle());
    return (error < 1);
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var output = pid.calculate(getAngle(), setpointAngle);
    motor.setVoltage(output);
  }
}
