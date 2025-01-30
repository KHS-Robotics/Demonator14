package frc.robot.subsystems.coraller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConfig;

public class Coraller extends SubsystemBase {
  public enum Configuration {
    STOW(CorallerConstants.STOW_HEIGHT, CorallerConstants.STOW_ANGLE),
    L1(CorallerConstants.L1_HEIGHT, CorallerConstants.L1_ANGLE),
    L2(CorallerConstants.L2_HEIGHT, CorallerConstants.L2_ANGLE),
    L3(CorallerConstants.L3_HEIGHT, CorallerConstants.L3_ANGLE),
    L4(CorallerConstants.L4_HEIGHT, CorallerConstants.L4_ANGLE),
    RECEIVE(CorallerConstants.RECEIVE_HEIGHT, CorallerConstants.RECEIVE_ANGLE);

    private final double elevatorPosition;
    private final double anglerPosition;

    private Configuration(double elevatorPosition, double anglerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.anglerPosition = anglerPosition;
    }
  }

  private final Elevator elevator;
  private final Angler angler;
  private final Intake intake;

  public Coraller() {
    elevator = new Elevator(ElevatorConfig.kRobotElevatorStowHeightInches);
    angler = new Angler();
    intake = new Intake();
  }

  public Command prepareToScore(Configuration cfg) {
    return Commands.parallel(
      this.runOnce(() -> elevator.setPosition(cfg.elevatorPosition)),
      this.runOnce(() -> angler.setPosition(cfg.anglerPosition))
    );
  }

  public Command intakeCoral() {
    return this.runOnce(() -> intake.intake());
  }

  public Command releaseCoral() {
    return this.runOnce(() -> intake.release());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
