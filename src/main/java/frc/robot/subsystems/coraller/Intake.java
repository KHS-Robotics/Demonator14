package frc.robot.subsystems.coraller;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

class Intake extends SubsystemBase {
  private boolean intaking, outaking;

  private final SparkMax motor;
  private final SparkLimitSwitch sensor;

  public Intake() {
    super(Coraller.class.getSimpleName() + "/" + Intake.class.getSimpleName());
    
    var intakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      // TODO: set inverted based on our desired sign of direction (positive intake / negative outake)
      .inverted(false);
    motor = new SparkMax(RobotMap.CORALLER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    motor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    sensor = motor.getForwardLimitSwitch();

    SmartDashboard.putData(getName(), this);
  }

  public void stop() {
    outaking = intaking = false;
    motor.stopMotor();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopIntake");
  }

  public void start() {
    outaking = false;
    intaking = true;
    // TODO: test for a good intake voltage
    motor.setVoltage(6);
  }

  public Command intakeCommand() {
    var cmd = runOnce(this::start);
    return cmd.withName("StartIntake");
  }

  public void reverse() {
    outaking = true;
    intaking = false;
    // TODO: test for a good reverse voltage
    motor.setVoltage(-6);
  }

  public Command reverseCommand() {
    var cmd = runOnce(this::reverse);
    return cmd.withName("ReverseIntake");
  }

  public boolean hasCoral() {
    return sensor.isPressed();
  }
  
  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addBooleanProperty("HasCoral", this::hasCoral, null);
    builder.addBooleanProperty("Intaking", () -> intaking, null);
    builder.addBooleanProperty("Outaking", () -> outaking, null);
  }
}
