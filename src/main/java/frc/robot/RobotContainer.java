// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.HIDConfig;
import frc.robot.Constants.LimelightConfig;
import frc.robot.Constants.PhotonVisionConfig;
import frc.robot.hid.DemonCommandXboxController;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.algae.collector.AlgaeCollector;
import frc.robot.subsystems.cameras.DemonLimelightCamera;
import frc.robot.subsystems.cameras.DemonPhotonCamera;
import frc.robot.subsystems.coraller.Coraller;
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
  public static final DemonCommandXboxController kDriverController = new DemonCommandXboxController(RobotMap.XBOX_PORT);
  public static final OperatorStick kOperatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

  // Subsystems - Mechanisms
  public static final SwerveDrive kSwerveDrive = new SwerveDrive();
  public static final Coraller kCoraller = new Coraller();
  public static final AlgaeCollector kAlgaeCollector = new AlgaeCollector();

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
    configureSubsystemDefaultCommands();
    configureBindings();
    configureAutonomous();

    SmartDashboard.putData(kField);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html#default-commands
   */
  private void configureSubsystemDefaultCommands() {
    // control swerve drive with the xbox controller by default
    kSwerveDrive.setDefaultCommand(kSwerveDrive.driveWithXboxController(kDriverController, () -> true,
        HIDConfig.kJoystickDeadband, HIDConfig.kJoystickSensitivity));

    // LowerFrontPhotonCamera - AprilTag updates for odometry
    kLowerFrontPhotonCamera.setDefaultCommand(kLowerFrontPhotonCamera.pollForPoseUpdates(
        (update) -> kSwerveDrive.addVisionMeasurementForOdometry(update.estimatedRobotPose.estimatedPose.toPose2d(),
            update.estimatedRobotPose.timestampSeconds, update.stdDevs)));

    // RearLimelightCamera - AprilTag updates for odometry
    kRearLimelightCamera.setDefaultCommand(
        kRearLimelightCamera
            .pollForPoseUpdates((estimate) -> kSwerveDrive.addVisionMeasurementForOdometry(estimate.pose,
                estimate.timestampSeconds, SwerveDrive.kDefaultVisionMeasurementStdDevs)));
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
   */
  private void configureBindings() {
    configureAutomatedBindings();
    configureXboxControllerBindings();
    configureOpertatorStickBindings();
  }

  /** Automated bindings that happen without pressing any buttons. */
  private void configureAutomatedBindings() {
  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {
    // reset robot heading to face away from the driver - this is useful during
    // driver practice to reset for field oriented driving direction or a rare odd
    // scenario on the field during a match
    kDriverController.resetRobotHeading().onTrue(kSwerveDrive.resetHeading());
  }

  /** Binds commands to operator stick buttons. */
  private void configureOpertatorStickBindings() {
  }

  /** https://pathplanner.dev/home.html */
  private void configureAutonomous() {
    // named commands for PathPlanner created auton routines
    configureNamedCommandsForAuto();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name from
    // PathPlanner
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    // add auton chooser to the Dashboard GUI for the drivers
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // PathPlanner logging for the current robot pose, trajectory and target pose
    // when running PathPlanner paths
    configurePathPlannerLogging();
  }

  /** https://pathplanner.dev/pplib-named-commands.html */
  private void configureNamedCommandsForAuto() {
    // String names here must match what is used in the PathPlanner GUI in order to
    // work properly!

    // stop all
    NamedCommands.registerCommand("STOP", kSwerveDrive.stopCommand().andThen(kCoraller.stopCommand()).andThen(kAlgaeCollector.stopCommand()));

    // Swerve Drive
    NamedCommands.registerCommand("STOPSwerve", kSwerveDrive.stopCommand());

    // Coraller
    NamedCommands.registerCommand("STOPCoraller", kCoraller.stopCommand());
    NamedCommands.registerCommand("PrepareStow", kCoraller.stow());
    NamedCommands.registerCommand("PrepareScoreL1", kCoraller.scoreL1());
    NamedCommands.registerCommand("PrepareScoreL2", kCoraller.scoreL2());
    NamedCommands.registerCommand("PrepareScoreL3", kCoraller.scoreL3());
    NamedCommands.registerCommand("PrepareScoreL4", kCoraller.scoreL4());
    NamedCommands.registerCommand("PreapareReceive", kCoraller.receive());
    NamedCommands.registerCommand("IntakeCoral", kCoraller.intakeCoral());
    NamedCommands.registerCommand("OuttakeCoral", kCoraller.outtakeCoral());

    // Algae
    NamedCommands.registerCommand("STOPAlgae", kAlgaeCollector.stopCommand());
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
