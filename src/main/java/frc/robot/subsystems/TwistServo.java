package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwistServo extends SubsystemBase {
  Servo twist = new Servo(0);

  public void latch(){
    twist.setAngle(90);

  }

  public void unlatch(){
    twist.setAngle(180);

  }
  
}
