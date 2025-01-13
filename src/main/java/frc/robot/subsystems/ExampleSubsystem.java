package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    private SparkMax exampleMotor;

    public ExampleSubsystem() {
        // Create motor configuration
        // https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/sparkbaseconfig
        // below are a few example configuration options for the motor
        SparkMaxConfig exampleMotorConfig = new SparkMaxConfig();
        exampleMotorConfig.idleMode(IdleMode.kBrake);
        exampleMotorConfig.inverted(true);
        exampleMotorConfig.smartCurrentLimit(40);
        // set any other configurations here...
        
        // Create and configure the motor as Brushless (NEO motor)
        // https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkmax
        this.exampleMotor = new SparkMax(0, MotorType.kBrushless);
        this.exampleMotor.configure(exampleMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
}