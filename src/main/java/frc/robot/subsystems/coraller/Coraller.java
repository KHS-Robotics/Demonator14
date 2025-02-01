package frc.robot.subsystems.coraller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CorallerConfig;

public class Coraller extends SubsystemBase {
  private final Elevator elevator = new Elevator();
  private final Angler angler = new Angler();
  private final Intake intake = new Intake();

  public Command reconfigure(Configuration cfg) {
    return runOnce(() -> Commands.parallel(
      elevator.setPosition(cfg.elevatorPosition),
      angler.setPosition(cfg.anglerPosition)
    ));
  }

  public Command intakeCoral() {
    return startEnd(
      intake::start,
      intake::stop
    )
    .until(intake::hasCoral) 
    .withTimeout(3);
  }

  public Command releaseCoral() {
    return startEnd(
      intake::reverse,
      intake::stop
    )
    .withTimeout(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum Configuration {
    STOW(CorallerConfig.STOW_HEIGHT, CorallerConfig.STOW_ANGLE),
    L1(CorallerConfig.L1_HEIGHT, CorallerConfig.L1_ANGLE),
    L2(CorallerConfig.L2_HEIGHT, CorallerConfig.L2_ANGLE),
    L3(CorallerConfig.L3_HEIGHT, CorallerConfig.L3_ANGLE),
    L4(CorallerConfig.L4_HEIGHT, CorallerConfig.L4_ANGLE),
    RECEIVE(CorallerConfig.RECEIVE_HEIGHT, CorallerConfig.RECEIVE_ANGLE);

    private final double elevatorPosition;
    private final double anglerPosition;

    private Configuration(double elevatorPosition, double anglerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.anglerPosition = anglerPosition;
    }
  }
}
