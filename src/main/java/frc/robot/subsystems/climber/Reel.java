package frc.robot.subsystems.climber;

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

  private final SparkMax reel;
  private ReelState reelState = ReelState.OFF;

  public Reel() {
    super(Climber.class.getSimpleName() + "/" + Reel.class.getSimpleName());

    var reelConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        // TODO: invert for our desired sign (positive to reelIn)
        .inverted(false);
    reel = new SparkMax(RobotMap.CLIMBER_REEL_ID, MotorType.kBrushless);
    reel.configure(reelConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SmartDashboard.putData(this);
  }

  public Command reelIn() {
    var climbAlgae = RobotContainer.kAlgaeCollector.climb();
    // TODO: full speed once we know direction???
    var reelIn = startEnd(() -> setReel(6), this::stop);
    var cmd = Commands.sequence(climbAlgae, reelIn);
    return cmd.withName("ClimberReelIn");
  }

  public Command reelOut() {
    var climbAlgae = RobotContainer.kAlgaeCollector.climb();
    // TODO: full speed once we know direction???
    var reelOut = startEnd(() -> setReel(-6), this::stop);
    var cmd = Commands.sequence(climbAlgae, reelOut);
    return cmd.withName("ClimberReelOut");
  }

  private void setReel(double voltage) {
    reelState = voltage > 0 ? ReelState.REELING_IN : voltage < 0 ? ReelState.REELING_OUT : ReelState.OFF;
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
    builder.addStringProperty("ReelState", reelState::toString, null);
  }

  private enum ReelState {
    OFF("Off"),
    REELING_IN("Reeling In"),
    REELING_OUT("Reeling Out");

    private final String state;

    private ReelState(String s) {
      state = s;
    }

    public String toString() {
      return this.state;
    }
  }

}
