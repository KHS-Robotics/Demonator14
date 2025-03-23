package frc.robot.subsystems.coraller;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coraller extends SubsystemBase {
  private final Elevator elevator = new Elevator();
  private final Angler angler = new Angler();
  private final Intake intake = new Intake(() -> angler.hasCoral());

  public Coraller() {
    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  public boolean hasCoral() {
    return angler.hasCoral();
  }

  public Command stow() {
    var cmd = setState(CorallerState.STOW);
    return cmd;
  }

  public Command receive() {
    var cmd = setState(CorallerState.RECEIVE);
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

  public Command scoreL3() {
    var cmd = setState(CorallerState.L3);
    return cmd;
  }

  public Command scoreL4() {
    var cmd = setState(CorallerState.L4);
    return cmd;
  }

  private Command setState(CorallerState state) {
    var cmd = Commands.parallel(
      elevator.setHeightCommand(state.elevatorPosition),
      angler.setAngleCommand(state.anglerPosition)
    );
    cmd.addRequirements(this);
    return cmd.withName("SetCorallerState(\"" + state.toString() + "\")");
  }

  public Command setElevatorOverride(boolean override) {
    return elevator.setOverride(override);
  }

  public Command intakeCoral(boolean ignoreSensor) {
    var cmd = startEnd(intake::start, intake::stop);
    cmd.addRequirements(intake);
    return cmd
      .until(() -> intake.hasCoral() && !ignoreSensor)
      .withName("IntakeCoral");
  }

  public Command intakeCoralToPrepareForStation() {
    var cmd = runOnce(intake::start);
    cmd.addRequirements(intake);
    return cmd
      .withName("IntakeCoral");
  }

  public Command outtakeCoral() {
    var cmd = startEnd(intake::reverse, intake::stop);
    cmd.addRequirements(intake);
    return cmd.withName("ReleaseCoral");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    cmd.addRequirements(elevator, angler, intake);
    return cmd.withName("StopCoraller");
  }

  public void stop() {
    elevator.stop();
    angler.stop();
    intake.stop();
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
