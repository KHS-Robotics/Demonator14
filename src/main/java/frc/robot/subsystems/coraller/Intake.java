package frc.robot.subsystems.coraller;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotMap;

class Intake {
  private final SparkMax intake;

  public Intake() {
    var intakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      .inverted(false);
    intake = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    intake.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);
  }

  // TODO() find out volts and which are inversed
  public void start() {
    intake.setVoltage(6);
  }

  public void reverse() {
    intake.setVoltage(-6);
  }
  
  public void stop() {
    intake.setVoltage(0);
  }

}
