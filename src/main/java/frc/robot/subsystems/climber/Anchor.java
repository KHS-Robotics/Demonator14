package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.climber.ClimberConfig.AnchorConfig;
import frc.robot.subsystems.climber.ClimberSetpoints.AnchorSetpoints;

public class Anchor extends SubsystemBase {
  private double currentSetpoint;

  private final SparkFlex anchor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public Anchor() {
    super(Climber.class.getSimpleName() + "/" + Anchor.class.getSimpleName());

    var encoderConfig = new EncoderConfig()
      .positionConversionFactor(AnchorConfig.kAnchorEncoderPositionConversionFactor)
      .velocityConversionFactor(AnchorConfig.kAnchorEncoderVelocityConversionFactor);
    
    var maxOutputPercentage = 0.05;
    var pidConfig = new ClosedLoopConfig()
      .pid(AnchorConfig.kAnchorP, 0, AnchorConfig.kAnchorD, ClosedLoopSlot.kSlot0)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 360)
      .outputRange(-maxOutputPercentage, maxOutputPercentage);
    var anchorConfig = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(10)
      .inverted(false)
      .apply(encoderConfig)
      .apply(pidConfig);
    anchor = new SparkFlex(RobotMap.CLIMBER_ANCHOR_ID, MotorType.kBrushless);
    anchor.configure(anchorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    encoder = anchor.getEncoder();
    pid = anchor.getClosedLoopController();

    SmartDashboard.putData(this);
  }

  public void periodic() {
  }

  public Command engageAnchor() {
    var cmd = run(() -> setSetpoint(AnchorSetpoints.kEngage));
    return cmd.withName("EngageAnchor");
  }

  public Command disengageAnchor() {
    var cmd = run(() -> setSetpoint(AnchorSetpoints.kDisengage));
    return cmd.withName("DisengageAnchor");
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(currentSetpoint - getAngle());
    return (error < 1);
  }

  private void setSetpoint(double setpoint) {
    currentSetpoint = setpoint;
    pid.setReference(setpoint, ControlType.kPosition);
  }

  public void stop() {
    anchor.stopMotor();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addBooleanProperty("isAtSetpoint", () -> this.isAtSetpoint(), null);
    builder.addDoubleProperty("Angle", () -> this.getAngle(), null);
    builder.addDoubleProperty("Setpoint", () -> currentSetpoint, this::setSetpoint);
  }
}

