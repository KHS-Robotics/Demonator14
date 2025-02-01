package frc.robot.subsystems.coraller;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

class Intake extends SubsystemBase{
  private final SparkMax motor;
  private final SparkLimitSwitch sensor;

  public Intake() {
    var intakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      .inverted(false);
    motor = new SparkMax(RobotMap.CORALLER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    motor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);

    sensor = motor.getForwardLimitSwitch();

    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addBooleanProperty("hasCoral", this::hasCoral, null);
    builder.addBooleanProperty("MotorIsOn", this::isMotorOn, null);
  }

  // TODO() find out volts and which are inversed
  public void start() {
    motor.setVoltage(6);
  }
  
  public void reverse() {
    motor.setVoltage(-6);
  }
  
  public void stop() {
    motor.setVoltage(0);
  }

  //false if motor is off
  public boolean isMotorOn(){
    return !(motor.get() == 0);
  }

  public boolean hasCoral() {
    return sensor.isPressed();
  }
}
