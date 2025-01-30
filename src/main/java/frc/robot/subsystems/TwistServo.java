package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Testing servo */
public class TwistServo extends SubsystemBase {
  Servo twist = new Servo(0);

  public Command latch() {
    return this.runOnce(() -> twist.setAngle(90));
  }

  public Command unlatch() {
    return this.runOnce(() -> twist.setAngle(180));
  }
}
