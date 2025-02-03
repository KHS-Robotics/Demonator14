package frc.robot.subsystems.algae.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {
  private final Intake intake = new Intake();
  private final Wrist wrist = new Wrist();

  public Command position(double pos) {
    return startEnd(
        () -> wrist.setAngleCommand(pos),
        wrist::stop
      )
      .withName("PositionWrist");
  }

  public Command intakeAlgae() {
    return startEnd(intake::start, intake::stop)
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
