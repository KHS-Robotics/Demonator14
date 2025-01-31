package frc.robot.subsystems.coraller;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotMap;

class Intake {
  private final SparkMax motor;
  private final SparkLimitSwitch sensor;

  public Intake() {
    var intakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      .inverted(false);
    motor = new SparkMax(RobotMap.CORALLER_INTAKE_ID, MotorType.kBrushless);
    motor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);

    sensor = motor.getForwardLimitSwitch();
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

  public boolean hasCoral() {
    return sensor.isPressed();
  }
}
