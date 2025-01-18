// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  /**
   * Gets the selected autonomous routine from SmartDashboard.
   * 
   * @return the selected autonomous routine from the dropdown in SmartDashboard
   */
  public Command getAutonomousCommand() {
    var selected = autoChooser.getSelected();
    return selected != null ? selected
        : new InstantCommand(() -> DriverStation.reportWarning("Null autonomous was selected.", false));
  }

  public static final AHRS navx = new AHRS(NavXComType.kUSB1);
  public static final Field2d field = new Field2d();

  // Human Interface Devices (HIDs)
  public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonomous();
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
  }

  /** Configures any autonomous resources. */
  private void configureAutonomous() {
    SmartDashboard.putData(field);

    configureNamedCommandsForAuto();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    this.configurePathPlannerLogging();
  }

  private void configureNamedCommandsForAuto() {
    NamedCommands.registerCommand("StopSwerve", new InstantCommand(() -> swerveDrive.stop(), swerveDrive));
  }

  private void configurePathPlannerLogging() {
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
