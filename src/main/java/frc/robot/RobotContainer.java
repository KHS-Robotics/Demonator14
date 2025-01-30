// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.LimelightConfig;
import frc.robot.Constants.PhotonVisionConfig;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.cameras.DemonLimelightCamera;
import frc.robot.subsystems.cameras.DemonPhotonCamera;
import frc.robot.subsystems.coraller.Coraller;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.TwistServo;


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
 * 
 * <p>
 * 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html
 */
public class RobotContainer {
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * Gets the selected autonomous routine from the Dashboard GUI.
   * <p>
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
   * 
   * @return the selected autonomous routine from the dropdown in the Dashboard
   *         GUI
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  // Widget for the Dashboard GUI to view the robot's position and heading on the
  // field.
  // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
  public static final Field2d kField = new Field2d();

  // Gyro / IMU (Inertial Measurement Unit) for Robot Orientation in 3-D space
  // https://docs.wpilib.org/en/stable/docs/hardware/sensors/gyros-hardware.html
  // https://pdocs.kauailabs.com/navx-mxp/
  // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
  public static final AHRS kNavx = new AHRS(NavXComType.kUSB1);

  // Operator / Human Interface Devices (HIDs)
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html
  public static final CommandXboxController kDriverController = new CommandXboxController(RobotMap.XBOX_PORT);
  public static final OperatorStick kOperatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

  // Subsystems - Mechanisms
  public static final SwerveDrive kSwerveDrive = new SwerveDrive();
  public static final Coraller kCoraller = new Coraller();

  // temporary
  public static final TwistServo kCageTwist = new TwistServo();

  // Subsystems - Cameras
  public static final DemonPhotonCamera kLowerFrontPhotonCamera = new DemonPhotonCamera(
      PhotonVisionConfig.kLowerFrontCameraName, PhotonVisionConfig.kRobotToLowerFrontCamera);
  public static final DemonLimelightCamera kRearLimelightCamera = new DemonLimelightCamera(
      LimelightConfig.kRearCameraName, LimelightConfig.kPoseAlgorithm, kSwerveDrive::getPose, kNavx::getRate);

  /**
   * The container for the robot. Contains subsystems, operator interface devices,
   * and commands.
   */
  public RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonomous();

    kSwerveDrive.resetPose(new Pose2d(8.5, 4.0, Rotation2d.fromDegrees(0)));
    SmartDashboard.putData(kField);
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html#default-commands
   */
  private void configureSubsystemDefaultCommands() {
    kSwerveDrive.setDefaultCommand(new DriveSwerveWithXbox(true, Constants.kJoystickSensitivity));
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
   */
  private void configureBindings() {
    this.configureAutomatedBindings();
    this.configureXboxControllerBindings();
    this.configureOpertatorStickBindings();
  }

  /** Automated bindings that happen without pressing any buttons. */
  private void configureAutomatedBindings() {

  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {
    kDriverController.start().onTrue(new InstantCommand(() -> {
      var currentPose = kSwerveDrive.getPose();
      var currentAlliance = DriverStation.getAlliance();
      var awayAngle = currentAlliance.isPresent() && currentAlliance.get().equals(Alliance.Red)
          ? 180
          : 0;
      kSwerveDrive.resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(awayAngle)));
    }, kSwerveDrive));

    // servo testing
    kDriverController.a().onTrue(new InstantCommand(() -> kCageTwist.latch(), kCageTwist));
    kDriverController.b().onTrue(new InstantCommand(() -> kCageTwist.unlatch(), kCageTwist));
    System.out.println("Xbox Controller Bound");

  }

  /** Binds commands to operator stick buttons. */
  private void configureOpertatorStickBindings() {
  }

  /** https://pathplanner.dev/home.html */
  private void configureAutonomous() {
    configureNamedCommandsForAuto();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    this.configurePathPlannerLogging();
  }

  // /** https://pathplanner.dev/pplib-named-commands.html */
  private void configureNamedCommandsForAuto() {
    NamedCommands.registerCommand("StopSwerve", new InstantCommand(() -> kSwerveDrive.stop(), kSwerveDrive));
  }

  // /** https://pathplanner.dev/pplib-custom-logging.html */
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
