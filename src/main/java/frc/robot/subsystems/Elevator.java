// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.EncoderConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;
// import frc.robot.Constants.ElevatorConfig;

// public class Elevator extends SubsystemBase {
//   public enum ElevatorPosition {
//     kStow(48),
//     kLevel1(0),
//     kLevel2(0),
//     kLevel3(0),
//     kLevel4(72),
//     kCoralStation(0);

//     /**
//      * inches
//      */
//     public final double height;

//     ElevatorPosition(double height) {
//       this.height = height;
//     }
//   }

//   private ElevatorPosition currentSetpoint = ElevatorPosition.kStow;

//   private final SparkMax leader;
//   private final RelativeEncoder elevatorEncoder;
//   private final SparkClosedLoopController elevatorPID;

//   public Elevator() {
//     var elevatorEncoderConfig = new EncoderConfig()
//         .positionConversionFactor(ElevatorConfig.kElevatorEncoderPositionConversionFactor)
//         .velocityConversionFactor(ElevatorConfig.kElevatorEncoderVelocityConversionFactor);
//     var elevatorClosedLoopConfig = new ClosedLoopConfig()
//         .pid(ElevatorConfig.kElevatorP, ElevatorConfig.kElevatorI, ElevatorConfig.kElevatorD, ClosedLoopSlot.kSlot0);
//     var elevatorConfig = new SparkMaxConfig()
//         .idleMode(IdleMode.kBrake)
//         .smartCurrentLimit(45)
//         .inverted(false)
//         .apply(elevatorEncoderConfig)
//         .apply(elevatorClosedLoopConfig);
//     leader = new SparkMax(RobotMap.ELEVATOR_DRIVE_ID, MotorType.kBrushless);
//     leader.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters,
//         SparkBase.PersistMode.kPersistParameters);
//     elevatorPID = leader.getClosedLoopController();
//     elevatorEncoder = leader.getEncoder();
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public double getHeightFromGround() {
//     return elevatorEncoder.getPosition() + ElevatorConfig.kRobotElevatorStowHeightInches;
//   }

//   // sets height relative to the floor
//   public void setPosition(ElevatorPosition position) {
//     currentSetpoint = position;
//     double setpoint = position.height - ElevatorConfig.kRobotElevatorStowHeightInches;
//     setSetpoint(setpoint);
//   }

//   // set height relative to bottom of elevator
//   public void setSetpoint(double setpoint) {
//     elevatorPID.setReference(setpoint, ControlType.kPosition);
//   }

//   // TODO() add once we know what sensor we are using
//   public boolean isElevatorAtBottom() {
//     return true;
//   }

//   public ElevatorPosition getSetpoint() {
//     return currentSetpoint;
//   }

//   public boolean isAtSetpoint() {
//     var error = Math.abs(currentSetpoint.height - getHeightFromGround());
//     return (error < 1);
//   }

//   // get height relative to bottom of elevator
//   public double getRelativePosition() {
//     return elevatorEncoder.getPosition();
//   }
// }
