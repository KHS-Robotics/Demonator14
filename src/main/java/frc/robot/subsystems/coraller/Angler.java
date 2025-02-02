package frc.robot.subsystems.coraller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorallerConfig;
import frc.robot.RobotMap;

class Angler extends SubsystemBase {
  private double setpointAngle;

  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final PIDController pid;

  public Angler() {
    super("Coraller/Angler");
    var anglerConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      // TODO: set inverted based on our desired sign of direction (positive up / negative down)
      .inverted(false);
    motor = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    motor.configure(anglerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    
    encoder = motor.getAbsoluteEncoder();

    pid = new PIDController(CorallerConfig.kAnglerP, CorallerConfig.kAnglerI, CorallerConfig.kAnglerD);
    pid.setIZone(5);
  
    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName()+"/PID Controller", pid);  
  }

  public void periodic() {
    setMotorOutputForSetpoint();
  }

  public Command setSetpointCommand(double angleDegrees) {
    return this.run(() -> setSetpoint(angleDegrees))
      .until(this::isAtSetpoint)
      .withName("SetAnglerSetpoint");
  }

  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopAngler");
  }

  /** 
   * This method sets the configured setpoint for the elevator. The periodic function
   * is contantly polling this value to make adjustments when it changes
   */
  public void setSetpoint(double setpoint) {
    // only reset for new setpoints
    if (setpoint != setpointAngle) {
      pid.reset();
    }
    setpointAngle = setpoint;
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngle - getAngle());
    return (error < 1);
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getAngle(), setpointAngle);
    var ffGravity = CorallerConfig.kAnglerKG * Math.cos(getAngle());
    var output = pidOutput + ffGravity;
    motor.setVoltage(output);
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Setpoint", () -> setpointAngle, this::setSetpoint);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("IsAtSetpoint", this::isAtSetpoint, null);
  }
}
