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

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 * 
 * <p>
 * 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  /**
   * Gets the selected autonomous routine from SmartDashboard.
   * <p>
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
   * 
   * @return the selected autonomous routine from the dropdown in SmartDashboard
   */
  public Command getAutonomousCommand() {
    var selected = autoChooser.getSelected();
    return selected != null ? selected
        : new InstantCommand(() -> DriverStation.reportWarning("Null autonomous was selected.", false));
  }

  // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
  public static final AHRS kNavx = new AHRS(NavXComType.kUSB1);

  // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
  public static final Field2d kField = new Field2d();

  // Human Interface Devices (HIDs)
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html
  public static final CommandXboxController kDriverController = new CommandXboxController(RobotMap.XBOX_PORT);

  // Subsystems
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
  public static final SwerveDrive kSwerveDrive = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonomous();
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html#default-commands
   */
  private void configureSubsystemDefaultCommands() {
    kSwerveDrive.setDefaultCommand(new DriveSwerveWithXbox(true, Constants.DRIVE_JOYSTICK_SENSITIVITY));
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
   */
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

  /** https://pathplanner.dev/home.html */
  private void configureAutonomous() {
    SmartDashboard.putData(kField);

    configureNamedCommandsForAuto();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    this.configurePathPlannerLogging();
  }

  /** https://pathplanner.dev/pplib-named-commands.html */
  private void configureNamedCommandsForAuto() {
    NamedCommands.registerCommand("StopSwerve", new InstantCommand(() -> kSwerveDrive.stop(), kSwerveDrive));
  }

  /** https://pathplanner.dev/pplib-custom-logging.html */
  private void configurePathPlannerLogging() {
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      kField.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      kField.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      kField.getObject("path").setPoses(poses);
    });
  }
}
