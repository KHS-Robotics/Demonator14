package frc.robot.subsystems.coraller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CorallerConfig;
import frc.robot.RobotMap;

public class Angler {
  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController pid;

  private double setPointAngle;

  public Angler() {
    var anglerClosedLoopConfig = new ClosedLoopConfig()
      .pid(CorallerConfig.kCorallerP, CorallerConfig.kCorallerI, CorallerConfig.kCorallerD,
          ClosedLoopSlot.kSlot0);
    var anglerConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      .inverted(false)
      .apply(anglerClosedLoopConfig);
    motor = new SparkMax(RobotMap.CORALLER_ANGLE_ID, MotorType.kBrushless);
    motor.configure(anglerConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getAbsoluteEncoder();
    pid = motor.getClosedLoopController();
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public void setPosition(double pos) {
    changeSetPoint(pos);
  }

  private void changeSetPoint(double setpoint) {
    pid.setReference(setpoint, ControlType.kPosition);
  }

  public double getSetPoint() {
    return setPointAngle;
  }

  public boolean isAtCorallerSetPoint() {
    var error = Math.abs(setPointAngle - getAngle());
    return (error < 1);
  }
}
