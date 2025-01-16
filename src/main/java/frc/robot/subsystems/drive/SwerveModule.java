/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Swerve Module
 */
public class SwerveModule extends SubsystemBase {
  public final String name;

  private boolean isFlipped;

  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final CANcoder pivotEncoder;
  private final SparkMax pivotMotor;
  private final SparkClosedLoopController drivePID;
  private final SimpleMotorFeedforward driveFeedForward;

  private final PIDController pivotPID;

  private double offsetAngle;

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   * @param driveP            P value of Drive PID
   * @param driveI            I value of Drive PID
   * @param driveD            D value of Drive PID
   * @param pivotEncoderId    port number for the pivot CANCoder
   * @param reversed          true if drive motor is reversed
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, boolean reversed, double offsetAngle) {

    this.name = name;
    setName("Module-" + name);

    var driveEncoderConfig = new EncoderConfig()
        .positionConversionFactor(Constants.DRIVE_POS_ENCODER)
        .velocityConversionFactor(Constants.DRIVE_VEL_ENCODER);
    var driveClosedLoopConfig = new ClosedLoopConfig()
        .pid(driveP, driveI, driveD, ClosedLoopSlot.kSlot0);
    var driveMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(reversed)
        .apply(driveEncoderConfig)
        .apply(driveClosedLoopConfig);
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    drivePID = driveMotor.getClosedLoopController();
    driveFeedForward = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);
    driveEncoder = driveMotor.getEncoder();

    var pivotMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    pivotMotor = new SparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.configure(pivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    pivotEncoder = new CANcoder(pivotEncoderId);
    pivotPID = new PIDController(pivotP, pivotI, pivotD);
    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(1);

    this.offsetAngle = offsetAngle;
  }

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, double offsetAngle) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, driveP, driveI,
        driveD, drivekS, drivekV, drivekA, pivotEncoderId, false, offsetAngle);
  }

  @Override
  public void periodic() {
    var state = getState();
    SmartDashboard.putNumber(this.name + "-Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(this.name + "-Angle", state.angle.getDegrees());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the PID values for the pivot module.
   *
   * @param p the p value for the pivot module
   * @param i the i value for the pivot module
   * @param d the d value for the pivot module
   */
  public void setPid(double p, double i, double d) {
    pivotPID.setPID(p, i, d);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state           desired state with the speed and angle
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    var targetAngle = useShortestPath ? calculateShortestPath(state.angle.getDegrees()) : state.angle.getDegrees();
    pivotMotor.set(MathUtil.clamp(pivotPID.calculate(getAngle(), targetAngle), -1, 1));

    var sign = isFlipped && useShortestPath ? -1 : 1;
    drivePID.setReference(sign * state.speedMetersPerSecond,
        SparkMax.ControlType.kVoltage, ClosedLoopSlot.kSlot0,
        driveFeedForward.calculate(sign * state.speedMetersPerSecond));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state desired state with the speed and angle
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed           the desired speed in meters/second
   * @param angle           the desired angle in degrees from [-180, 180]
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(double speed, double angle, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)), useShortestPath);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   */
  public void setDesiredState(double speed, double angle) {
    setDesiredState(speed, angle, true);
  }

  /**
   * Gets the angle of the pivot module.
   *
   * @return the angle of the pivot module ranging from [-180,180]
   */
  public double getAngle() {
    double voltage = pivotEncoder.getAbsolutePosition().getValueAsDouble();
    double angle = voltage * 360 + offsetAngle;

    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }

    return angle;
  }

  /**
   * Stops the module.
   */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPID.reset();
  }

  /**
   * Calculates the shortest path the pivot module should take, it
   * might be the given <code>targetAngle</code>. Flips the drive motor
   * if there is a shorter path.
   *
   * @param targetAngle the desired angle of the module
   * @return the shortest path to the target angle, flips the
   *         drive motor if there is a shorter path
   */
  private double calculateShortestPath(double targetAngle) {
    var currentAngle = this.getAngle();
    var dAngle = Math.abs(targetAngle - currentAngle);

    isFlipped = dAngle > 90 && dAngle < 270;

    if (isFlipped) {
      if (targetAngle > 0 || targetAngle == 0 && currentAngle < 0) {
        targetAngle -= 180;
      } else if (targetAngle < 0 || targetAngle == 0 && currentAngle > 0) {
        targetAngle += 180;
      }
    }

    return targetAngle;
  }
}
