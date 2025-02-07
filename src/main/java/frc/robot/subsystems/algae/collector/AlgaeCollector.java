package frc.robot.subsystems.algae.collector;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {
  private final Intake intake = new Intake();
  private final Wrist wrist = new Wrist();

  public AlgaeCollector() {
    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  public Command deploy() {
    var cmd = setState(AlgaeCollectorState.DEPLOY);
    return cmd;
  }

  public Command stow() {
    var cmd = setState(AlgaeCollectorState.STOW);
    return cmd;
  }

  private Command setState(AlgaeCollectorState state) {
    var cmd = wrist.setAngleCommand(state.wristAngle);
    cmd.addRequirements(this);
    return cmd.withName("SetAlgaeCollectorState(\"" + state.toString() + "\")");
  }

  public Command intakeAlgae() {
    var cmd = startEnd(intake::start, intake::stop).until(intake::hasAlgae);
    cmd.addRequirements(intake);
    return cmd.withName("IntakeAlgae");
  }

  public Command outtakeAlgae() {
    var cmd = startEnd(intake::reverse, intake::stop);
    cmd.addRequirements(intake);
    return cmd.withName("ReleaseAlgae");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopAlgaeCollector");
  }

  public void stop() {
    intake.stop();
    wrist.stop();
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
  }
}
