package frc.robot.subsystems.coraller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.subsystems.coraller.CorallerConfig.AnglerConfig;

class Angler extends SubsystemBase {
  private double setpointAngleDegrees;

  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final PIDController pid;

  public Angler() {
    super(Coraller.class.getSimpleName() + "/" + Angler.class.getSimpleName());

    var anglerConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false);
    motor = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    motor.configure(anglerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    pid = new PIDController(AnglerConfig.kAnglerP, AnglerConfig.kAnglerI, AnglerConfig.kAnglerD);
    pid.setIZone(5);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
  }

  /** {@inheritDoc} */
  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

  public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetAnglerSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopAngler");
  }

  /**
   * This method sets the configured setpoint for the elevator. The periodic
   * function is contantly polling this value to make adjustments when it changes
   * 
   * @see {@link #periodic()}
   */
  public void setSetpointAngle(double setpointDegrees) {
    // only reset for new setpoints
    if (setpointDegrees != setpointAngleDegrees) {
      pid.reset();
    }
    setpointAngleDegrees = setpointDegrees;
  }

  public double getAngle() {
    //0 is strait out
    return Units.rotationsToDegrees(encoder.getPosition()) + AnglerConfig.kAnglerOffset;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 1);
  }

  private void setMotorOutputForSetpoint() {
    // TODO: sysid characterization + feedforward terms
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);
    var ffGravity = AnglerConfig.kAnglerKG * Math.cos(Math.toRadians(getAngle()));
    var output = pidOutput + ffGravity;
    motor.setVoltage(output);
  }

  /** Updates the setpoint to the current position. */
  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointAngle(getAngle());
    }
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
  }

  /** {@inheritDoc} */
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
}
