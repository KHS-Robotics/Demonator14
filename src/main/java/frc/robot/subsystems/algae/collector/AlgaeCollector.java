package frc.robot.subsystems.algae.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {
  private final Intake intake = new Intake();
  private final Wrist wrist = new Wrist();

  public Command position(double pos) {
    return runOnce(() -> wrist.setAngleCommand(pos))
      .withName("PositionWrist");
  }

  public Command intakeAlgae() {
    var cmd = startEnd(intake::start, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .until(intake::hasAlgae)
      .withTimeout(3)
      .withName("IntakeAlgae");
  }

  public Command outtakeAlgae() {
    var cmd = startEnd(intake::reverse, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .withTimeout(.5)
      .withName("ReleaseAlgae");
  }

  public Command stopCommand() {
    return Commands.parallel(
      intake.stopCommand(),
      wrist.stopCommand()
    ).withName("StopAlgaeCollector");
  }

}
