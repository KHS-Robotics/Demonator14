package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.climber.ClimberConfig.AnchorConfig;
import frc.robot.subsystems.climber.ClimberSetpoints.AnchorSetpoints;
import frc.robot.subsystems.drive.SwerveDriveConfig;;

public class Anchor extends SubsystemBase {
  private double currentSetpoint;

  private final SparkMax anchor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public Anchor() {
    super(Climber.class.getSimpleName() + "/" + Anchor.class.getSimpleName());

    var encoderConfig = new EncoderConfig()
      .positionConversionFactor(AnchorConfig.kDriveEncoderPositionConversionFactor);
    var pidConfig = new ClosedLoopConfig()
      .pid(AnchorConfig.kAnchorP, 0, 0, ClosedLoopSlot.kSlot0);
    var anchorConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      .apply(encoderConfig)
      .apply(pidConfig)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false);
    anchor = new SparkMax(RobotMap.CLIMBER_ANCHOR_ID, MotorType.kBrushless);
    anchor.configure(anchorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    encoder = anchor.getEncoder();
    pid = anchor.getClosedLoopController();

    SmartDashboard.putData(this);
  }

  public void periodic(){
  }

  public Command engageAnchor() {
    var cmd = run(() -> setSetpoint(AnchorSetpoints.kAnchorEngaged)).until(this::isAtSetpoint);
    return cmd.withName("EngageAnchor");
  }

  public Command disengageAnchor() {
    var cmd = run(() -> setSetpoint(AnchorSetpoints.kAnchorUnengaged)).until(this::isAtSetpoint);
    return cmd.withName("UnengageAnchor");
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(currentSetpoint - getAngle());
    return (error < 3);
  }

  private void setSetpoint(double setpoint) {
    currentSetpoint = setpoint;
    pid.setReference(setpoint, ControlType.kPosition);
  }
}

