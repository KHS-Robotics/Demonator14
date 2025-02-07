package frc.robot.subsystems.coraller;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CorallerConfig;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConfig;

public class Coraller extends SubsystemBase {
  private final Elevator elevator = new Elevator();
  private final Angler angler = new Angler();
  private final Intake intake = new Intake();

  public Coraller() {
    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    updateSetpointsForDisabledMode();
  }

  public Command prepareToScoreReef(ReefScoringConfiguration cfg) {
    var cmd = Commands.parallel(
      elevator.setHeightCommand(cfg.elevatorPosition),
      angler.setAngleCommand(cfg.anglerPosition)
    );
    cmd.addRequirements(this);
    return cmd.withName("PrepareToScoreReef");
  }

  public Command intakeCoral() {
    var cmd = startEnd(intake::start, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .until(intake::hasCoral)
      .withTimeout(3)
      .withName("IntakeCoral");
  }

  public Command outtakeCoral() {
    var cmd = startEnd(intake::reverse, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .withTimeout(.5)
      .withName("ReleaseCoral");
  }

  public Command stopCommand() {
    var cmd = Commands.parallel(
      elevator.stopCommand(),
      angler.stopCommand(),
      intake.stopCommand()
    );
    cmd.addRequirements(this);
    return cmd.withName("StopCoraller");
  }

  public void stop() {
    elevator.stop();
    angler.stop();
    intake.stop();
  }

  /** Updates the setpoints to the current positions. */
  private void updateSetpointsForDisabledMode() {
    // only in disabled
    if (RobotState.isDisabled()) {
      // elevator - ensure non-negative
      var isElevatorEncoderNonNegative = elevator.getHeightFromBottomInches() >= 0;
      elevator.setSetpointHeight(isElevatorEncoderNonNegative ? elevator.getHeightFromGroundInches() : ElevatorConfig.STOW_HEIGHT);

      // angler
      angler.setSetpointAngle(angler.getAngle());
    }
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
  }

  /** Heights and angles to score on the reef. */
  public enum ReefScoringConfiguration {
    STOW(ElevatorConfig.STOW_HEIGHT, CorallerConfig.STOW_ANGLE),
    L1(ElevatorConfig.L1_HEIGHT, CorallerConfig.L1_ANGLE),
    L2(ElevatorConfig.L2_HEIGHT, CorallerConfig.L2_ANGLE),
    L3(ElevatorConfig.L3_HEIGHT, CorallerConfig.L3_ANGLE),
    L4(ElevatorConfig.L4_HEIGHT, CorallerConfig.L4_ANGLE),
    RECEIVE(ElevatorConfig.RECEIVE_HEIGHT, CorallerConfig.RECEIVE_ANGLE);

    /** Inches */
    private final double elevatorPosition;
    /** Degrees */
    private final double anglerPosition;

    private ReefScoringConfiguration(double elevatorPosition, double anglerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.anglerPosition = anglerPosition;
    }
  }
}
