// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// PIZZA IS THE BEST WITH EXTRA SAUCE
// pizza is the worst sauce
//Luke is also here

// hi - byee
//vgbhjikjuhbvfg
//go to github
// i think sirup butter is better then regular
//hi andrea
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private static RobotContainer instance;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  private SendableChooser<Command> autoChooser;

  /**
   * Gets the selected autonomous routine from SmartDashboard.
   * 
   * @return the selected autonomous routine from the dropdown in SmartDashboard
   */
  public Command getAutonomousCommand() {
    // get as a proxy command so we are free to compose a SequentialCommand group
    // from it while avoiding a runtime exception
    return new ProxyCommand(() -> {
      var selected = autoChooser.getSelected();
      return selected != null ? selected
          : new InstantCommand(() -> DriverStation.reportWarning("Null autonomous was selected.", false));
    }).andThen(new InstantCommand(() -> {
      RobotContainer.swerveDrive.stop();
      RobotContainer.shooter.stopIndexer();
      RobotContainer.shooter.stopShooting();
      RobotContainer.intake.stop();
    }, RobotContainer.swerveDrive, RobotContainer.shooter, RobotContainer.intake));
  }

  private AutoBuilder autoBuilder;

  public AutoBuilder getAutoBuilder() {
    return autoBuilder;
  }

  public static final AHRS navx = new AHRS(Port.kUSB);

  public static final Field2d field = new Field2d();

  // Human Interface Devices (HIDs)
  public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
  public static final OperatorStick operatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
  }

  /** Configures the subsystem's default commands. */
  private void configureSubsystemDefaultCommands() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox(true));
  }

  private void configureBindings() {
    this.configureAutomatedBindings();
    this.configureXboxControllerBindings();
   
  }

  /** Automated bindings that happen without pressing any buttons. */
  private void configureAutomatedBindings() {

  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {
    var resetHeading = driverController.start().debounce(0.25);
    resetHeading.onTrue(new InstantCommand(() -> {
      swerveDrive.resetNavx();
      var isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
      swerveDrive.setPose(new Pose2d(8, 4, isRedAlliance ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)));
    }, swerveDrive));

    var robotRelativeDrive = driverController.rightTrigger(0.5);
    robotRelativeDrive.whileTrue(new DriveSwerveWithXbox(false));

    var slowDrive = driverController.x();
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 1;
      SwerveDrive.kMaxSpeedMetersPerSecond = 1;
    }));
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 4.6;
    }));

    
  private Command straightenSwervesCommand() {
    return new RepeatCommand(
        new InstantCommand(() -> swerveDrive.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        }), swerveDrive))
        .withTimeout(1)
        .andThen(new InstantCommand(() -> swerveDrive.stop(), swerveDrive));
  }

  private void configurePathPlannerLogging() {
    SmartDashboard.putData("field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }
}