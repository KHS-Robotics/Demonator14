package frc.robot.subsystems.coraller;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase;
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
  private final BooleanSupplier hasCoral;

  public Intake(BooleanSupplier hasCoral) {
    super(Coraller.class.getSimpleName() + "/" + Intake.class.getSimpleName());
    
    var intakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30)
      .inverted(true);
    motor = new SparkMax(RobotMap.CORALLER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    motor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    this.hasCoral = hasCoral;

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
    motor.setVoltage(2.25);
  }

  public Command intakeCommand() {
    var cmd = runOnce(this::start);
    return cmd.withName("StartIntake");
  }

  public void reverse() {
    intakeState = IntakeState.OUTAKING;
    motor.setVoltage(-12);
  }

  public void reverseSlow() {
    intakeState = IntakeState.OUTAKING;
    motor.setVoltage(-5);
  }

  public Command outtakeCommand() {
    var cmd = runOnce(this::reverse);
    return cmd.withName("ReverseIntake");
  }

  public Command outtakeSlowCommand() {
    var cmd = runOnce(this::reverseSlow);
    return cmd.withName("ReverseIntakeSlow");
  }

  public boolean hasCoral() {
    return hasCoral.getAsBoolean();
  }
  
  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addStringProperty("IntakeState", () -> intakeState.toString(), null);
    builder.addBooleanProperty("HasCoral", () -> hasCoral(), null);
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
