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
  private IntakeState intakeState = IntakeState.IDLE;

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

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  public void stop() {
    intakeState = IntakeState.IDLE;
    motor.stopMotor();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopIntake");
  }

  public void start() {
    intakeState = IntakeState.INTAKING;
    // TODO: test for a good intake voltage
    motor.setVoltage(6);
  }

  public Command intakeCommand() {
    var cmd = runOnce(this::start);
    return cmd.withName("StartIntake");
  }

  public void reverse() {
    intakeState = IntakeState.OUTAKING;
    // TODO: test for a good reverse voltage
    motor.setVoltage(-6);
  }

  public Command outtakeCommand() {
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
    builder.addStringProperty("IntakeState", () -> intakeState.toString(), null);
  }

  public enum IntakeState {
    IDLE("Idle"),
    INTAKING("Intaking"),
    OUTAKING("Outaking");

    private final String state;

    private IntakeState(String s) {
      state = s;
    }

    public String toString() {
      return this.state;
    }
  }
}
