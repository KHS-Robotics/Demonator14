package frc.robot.subsystems.coraller;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.coraller.CorallerSetpoints.AnglerSetpoints;
import frc.robot.subsystems.coraller.CorallerSetpoints.ElevatorSetpoints;
import frc.robot.subsystems.coraller.CorallerSetpoints.FlickerSetpoints;

public class Coraller extends SubsystemBase {
  private final Elevator elevator = new Elevator();
  private final Angler angler = new Angler();
  private final Intake intake = new Intake();
  private final Flicker flicker = new Flicker();

  public Coraller() {
    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  public Command stow() {
    var cmd = setState(CorallerState.STOW);
    return cmd;
  }

  public Command scoreL1() {
    var cmd = setState(CorallerState.L1);
    return cmd;
  }

  public Command scoreL2() {
    var cmd = setState(CorallerState.L2);
    return cmd;
  }

  public Command algaeL2() {
    var cmd = setState(CorallerState.L2_ALGAE);
    return cmd;
  }

  public Command scoreL3() {
    var cmd = setState(CorallerState.L3);
    return cmd;
  }

  public Command algaeL3() {
    var cmd = setState(CorallerState.L3_ALGAE);
    return cmd;
  }

  public Command scoreL4() {
    var cmd = setState(CorallerState.L4);
    return cmd;
  }

  public Command receive() {
    var cmd = setState(CorallerState.RECEIVE);
    return cmd;
  }

  private Command setState(CorallerState state) {
    var cmd = Commands.parallel(
      elevator.setHeightCommand(state.elevatorPosition),
      angler.setAngleCommand(state.anglerPosition),
      flicker.setAngleCommand(state.flickerPosition)
    );
    cmd.addRequirements(this);
    return cmd.withName("SetCorallerState(\"" + state.toString() + "\")");
  }

  public Command deployFlickerL2() {
    var cmd = flicker.setAngleCommand(CorallerState.L2_ALGAE.flickerPosition);
    cmd.addRequirements(this);
    return cmd.withName("DeployFlickerL2");
  }

  public Command deployFlickerL3() {
    var cmd = flicker.setAngleCommand(CorallerState.L3_ALGAE.flickerPosition);
    cmd.addRequirements(this);
    return cmd.withName("DeployFlickerL3");
  }

  public Command retractFlicker() {
    var cmd = flicker.setAngleCommand(CorallerState.STOW.flickerPosition);
    cmd.addRequirements(this);
    return cmd.withName("RetractFlicker");
  }

  public Command intakeCoral() {
    var cmd = startEnd(intake::start, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .until(intake::hasCoral)
      .withTimeout(15)
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
    var cmd = runOnce(this::stop);
    return cmd.withName("StopCoraller");
  }

  public void stop() {
    elevator.stop();
    angler.stop();
    intake.stop();
    flicker.stop();
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
  }

  /** Heights and angles for the Coraller's possible states. */
  private enum CorallerState {
    STOW(ElevatorSetpoints.STOW_HEIGHT, AnglerSetpoints.STOW_ANGLE, FlickerSetpoints.STOW_ANGLE),
    L1(ElevatorSetpoints.L1_HEIGHT, AnglerSetpoints.L1_ANGLE, FlickerSetpoints.STOW_ANGLE),
    L2(ElevatorSetpoints.L2_HEIGHT, AnglerSetpoints.L2_ANGLE, FlickerSetpoints.STOW_ANGLE),
    L2_ALGAE(ElevatorSetpoints.L2_HEIGHT, AnglerSetpoints.L2_ANGLE, FlickerSetpoints.L2_ANGLE),
    L3(ElevatorSetpoints.L3_HEIGHT, AnglerSetpoints.L3_ANGLE, FlickerSetpoints.STOW_ANGLE),
    L3_ALGAE(ElevatorSetpoints.L3_HEIGHT, AnglerSetpoints.L3_ANGLE, FlickerSetpoints.L3_ANGLE),
    L4(ElevatorSetpoints.L4_HEIGHT, AnglerSetpoints.L4_ANGLE, FlickerSetpoints.STOW_ANGLE),
    RECEIVE(ElevatorSetpoints.RECEIVE_HEIGHT, AnglerSetpoints.RECEIVE_ANGLE, FlickerSetpoints.STOW_ANGLE);

    /** Inches */
    public final double elevatorPosition;
    /** Degrees */
    public final double anglerPosition;
    /** Degrees */
    public final double flickerPosition;

    private CorallerState(double elevatorPosition, double anglerPosition, double flickerPosition) {
      this.elevatorPosition = elevatorPosition;
      this.anglerPosition = anglerPosition;
      this.flickerPosition = flickerPosition;
    }
  }
}
