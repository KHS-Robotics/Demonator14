package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Reel extends SubsystemBase {
  private ReelState reelState = ReelState.OFF;
  private enum ReelState {
    OFF, REELING;
  }

  private final SparkMax reel;
  private final RelativeEncoder encoder;

  public Reel() {
    super(Climber.class.getSimpleName() + "/" + Reel.class.getSimpleName());

    var reelConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false)
        .closedLoopRampRate(0.5)
        .openLoopRampRate(0.5);
    reel = new SparkMax(RobotMap.CLIMBER_REEL_ID, MotorType.kBrushless);
    reel.configure(reelConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = reel.getEncoder();

    SmartDashboard.putData(this);
  }

  public Command start() {
    var deployAlgae = RobotContainer.kAlgaeCollector.deploy();
    var runReel = startEnd(() -> setReel(8), this::stop);
    var cmd = Commands.sequence(deployAlgae, runReel);
    return cmd.withName("ClimberStartReel");
  }

  private void setReel(double voltage) {
    reelState = voltage != 0 ? ReelState.REELING : ReelState.OFF;
    reel.setVoltage(voltage);
  }

  public void stop() {
    setReel(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addStringProperty("ReelState", () -> reelState.toString(), null);
    builder.addDoubleProperty("Position", () -> encoder.getPosition(), null);
  }
}
