package frc.robot.subsystems.coraller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConfig;

public class Coraller extends SubsystemBase {
  public enum Level {
    STOW(0, 0),
    L1(0, 0),
    L2(0, 0),
    L3(0, 0),
    L4(0, 0);

    private final double elevatorPosition;
    private final double anglerPosition;

    private Level(double elevatorPosition, double anglerPosition) {
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

  public Command prepareToScore(Level level) {
    return Commands.parallel(
      this.runOnce(() -> elevator.setPosition(level.elevatorPosition)),
      this.runOnce(() -> angler.setPosition(level.anglerPosition))
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
