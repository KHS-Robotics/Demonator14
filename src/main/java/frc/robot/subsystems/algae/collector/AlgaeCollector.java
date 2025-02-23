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
    var cmd = wrist.deploy();
    cmd.addRequirements(this);
    return cmd;
  }

  public Command stow() {
    var cmd = wrist.stow();
    cmd.addRequirements(this);
    return cmd;
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
    cmd.addRequirements(intake, wrist);
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
