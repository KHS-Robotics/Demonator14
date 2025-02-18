package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.climber.ClimberConfig.AnchorConfig;

class Anchor extends SubsystemBase {
  
  private final SparkMax anchor;
  private final RelativeEncoder encoder;
  private final PIDController pid;
  // private double setpointAngleDegrees;

  

  public Anchor(){
    var anchorConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      // TODO: set inverted based on our desired sign of direction (positive up /
      // negative down)
      .inverted(false);
    anchor = new SparkMax(RobotMap.CLIMBER_ANCHOR_ID, MotorType.kBrushless);
    anchor.configure(anchorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    encoder = anchor.getEncoder();

    pid = new PIDController(AnchorConfig.kAnchorP, AnchorConfig.kAnchorI, AnchorConfig.kAnchorD);

  }

  public void periodic(){
    setMotorOutputForSetpoint();
  }

  public Command engageAnchor(){
    //set anchor to 90 degrees
    //setpointAngleDegrees = 90;
  }

  public Command unengageAnchor(){
    //set anchor to 0 degrees
    //setpointAngleDegrees = 90;  
  }

 

  private void setMotorOutputForSetpoint(){
    var pidOutput = pid.calculate(encoder.getPosition(), /*setpointAngleDegrees */ );
    anchor.setVoltage(pidOutput);
    // idk about gravity stuff so not messing w/ it for now
  }
}

