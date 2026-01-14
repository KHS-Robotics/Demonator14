// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//happy new year gng

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.hid.DemonCommandXboxController;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.algae.collector.AlgaeCollector;
import frc.robot.subsystems.cameras.CameraConfig.PhotonVisionConfig;
import frc.robot.subsystems.cameras.CameraConfig.LimelightConfig;
import frc.robot.subsystems.cameras.CameraConfig;
import frc.robot.subsystems.cameras.DemonLimelightCamera;
import frc.robot.subsystems.cameras.DemonPhotonCamera;
import frc.robot.subsystems.coraller.Coraller;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.led.LEDStrip;

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
  public static final OperatorStick kOperatorStick = new OperatorStick(RobotMap.OPERATOR_STICK_PORT);

  // Subsystems
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

  // Subsystems - Mechanisms
  public static final SwerveDrive kSwerveDrive = new SwerveDrive();
  public static final Coraller kCoraller = new Coraller();
  public static final AlgaeCollector kAlgaeCollector = new AlgaeCollector();
  public static final Climber kClimber = new Climber();

  // Subsystems - Cameras
  // photon
  public static final DemonPhotonCamera kFrontRightPhotonCamera = new DemonPhotonCamera(
       PhotonVisionConfig.kFrontRightCameraName, PhotonVisionConfig.kRobotToFrontRightCamera);
  public static final DemonPhotonCamera kFrontLeftPhotonCamera = new DemonPhotonCamera(
        PhotonVisionConfig.kFrontLeftCameraName, PhotonVisionConfig.kRobotToFrontLeftCamera);
  public static final DemonPhotonCamera kFrontTopPhotonCamera = new DemonPhotonCamera(
        PhotonVisionConfig.kFrontTopCameraName, PhotonVisionConfig.kRobotToFrontTopCamera);
  // limelight
  public static final DemonLimelightCamera kRearLimelightCamera = new DemonLimelightCamera(
      LimelightConfig.kRearCameraName, LimelightConfig.kPoseAlgorithm, kSwerveDrive::getPose, kNavx::getRate);

  // Subsystems - LED indicators
  public static final LEDStrip kLedStrip = new LEDStrip(
    () -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontRightPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceReefFiducialIds : CameraConfig.kBlueAllianceReefFiducialIds).isPresent();
    },   
    () -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontLeftPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceReefFiducialIds : CameraConfig.kBlueAllianceReefFiducialIds).isPresent();
    },   
    () -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontTopPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceCoralFiducialIds : CameraConfig.kBlueAllianceCoralFiducialIds).isPresent();
    },   
    () -> kCoraller.hasCoral(),
    () -> kClimber.kAnchor.isEngaged()
  );

  /**
   * The container for the robot. Contains subsystems, operator interface devices,
   * and commands.
   */
  public RobotContainer() {
    configureSubsystemDefaultCommands();
    configureBindings();
    configureAutonomous();

    SmartDashboard.putData(kNavx);
    SmartDashboard.putData(kField);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html#default-commands
   */
  private void configureSubsystemDefaultCommands() {
    // control swerve drive with the xbox controller by default
    kSwerveDrive.setDefaultCommand(kSwerveDrive.driveWithXboxController(kDriverController, () -> !kDriverController.robotRelative().getAsBoolean(),
        DemonCommandXboxController.kJoystickDeadband, DemonCommandXboxController.kJoystickSensitivity));

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
    // reset robot heading - ALWAYS FACE RED ALLIANCE WHEN DOING THIS - this is
    // useful during driver practice to reset for field oriented driving direction
    // or a rare odd scenario on the field during a match
    kDriverController.resetRobotHeading().onTrue(kSwerveDrive.resetHeading());

    // give driver ability to limit speeds for when elevator is high up to
    // help prevent tipping over - useful for slight alignment adjustments too
    kDriverController.goSlow().whileTrue(kSwerveDrive.goSlow());

    // For defense / to make the robot harder to move
    kDriverController.lockHeadingForDefense()
      .whileTrue(kSwerveDrive.holdCurrentHeadingWhileDriving(kDriverController, () -> !kDriverController.robotRelative().getAsBoolean(),
        DemonCommandXboxController.kJoystickDeadband, DemonCommandXboxController.kJoystickSensitivity));

    // vision alignment
    // COMMENTED OUT FOR DEMOS - FOR SAFETY
    kDriverController.alignToScoreRight().whileTrue(kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontLeftPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceReefFiducialIds : CameraConfig.kBlueAllianceReefFiducialIds);
    }).repeatedly());

    kDriverController.alignToScoreLeft().whileTrue(kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontRightPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceReefFiducialIds : CameraConfig.kBlueAllianceReefFiducialIds);
    }).repeatedly());

    kDriverController.alignToCoralStation().whileTrue(kSwerveDrive.alignToCoralStation(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      return kFrontTopPhotonCamera.getBestAprilTag(alliance == Alliance.Red ? CameraConfig.kRedAllianceCoralFiducialIds : CameraConfig.kBlueAllianceCoralFiducialIds);
    }).repeatedly());
  }

  /** Binds commands to operator stick buttons. */
  private void configureOpertatorStickBindings() {
    // Coraller
    kOperatorStick.stowCoraller().onTrue(kCoraller.stow());
    kOperatorStick.receiveCoraller().onTrue(kCoraller.receive());
    kOperatorStick.scoreL1().onTrue(kCoraller.scoreL1());
    kOperatorStick.scoreL2().onTrue(kCoraller.scoreL2());
    kOperatorStick.scoreL3().onTrue(kCoraller.scoreL3());
    kOperatorStick.scoreL4().onTrue(kCoraller.scoreL4());
    kOperatorStick.outtakeCoral().whileTrue(kCoraller.outtakeCoral());
    kOperatorStick.intakeCoral().whileTrue(kCoraller.intakeCoral(false));
    kOperatorStick.outtakeForL4().whileTrue(kCoraller.intakeCoral(true));
    kOperatorStick.disableElevatorOverride().onTrue(kCoraller.setElevatorOverride(false));
    kOperatorStick.enableElevatorOverride().onTrue(kCoraller.setElevatorOverride(true));

    // Algae Collector
    kOperatorStick.stowAlgaeCollector().onTrue(kAlgaeCollector.stow());
    kOperatorStick.deployAlgaeCollector().onTrue(kAlgaeCollector.deploy());
    kOperatorStick.outtakeAlgae().whileTrue(kAlgaeCollector.outtakeAlgae());
    kOperatorStick.intakeAlgae().whileTrue(kAlgaeCollector.intakeAlgae());

    // Climber
    kOperatorStick.engageAnchor().onTrue(kClimber.kAnchor.engageAnchor());
    kOperatorStick.disengageAnchor().onTrue(kClimber.kAnchor.disengageAnchor());
    kOperatorStick.climb().whileTrue(kClimber.kReel.start());
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
    NamedCommands.registerCommand("STOP",
        Commands.sequence(kSwerveDrive.stopCommand(), kCoraller.stopCommand(), kAlgaeCollector.stopCommand())
            .withName("StopAll"));

    // Swerve Drive
    NamedCommands.registerCommand("STOPSwerve", kSwerveDrive.stopCommand());

    // Coraller
    NamedCommands.registerCommand("STOPCoraller", kCoraller.stopCommand());
    NamedCommands.registerCommand("PrepareStow", kCoraller.stow());
    NamedCommands.registerCommand("PrepareReceive", kCoraller.receive());
    NamedCommands.registerCommand("PrepareScoreL1", kCoraller.scoreL1());
    NamedCommands.registerCommand("PrepareScoreL2", kCoraller.scoreL2());
    NamedCommands.registerCommand("PrepareScoreL3", kCoraller.scoreL3());
    NamedCommands.registerCommand("PrepareScoreL4", kCoraller.scoreL4());
    NamedCommands.registerCommand("IntakeCoral", kCoraller.intakeCoral(false));
    NamedCommands.registerCommand("StartCoralIntake", kCoraller.intakeCoralToPrepareForStation());
    NamedCommands.registerCommand("L4OuttakeCoral", kCoraller.intakeCoral(true).withTimeout(0.5));
    NamedCommands.registerCommand("OuttakeCoral", kCoraller.outtakeCoral().withTimeout(0.5));

    // Algae
    NamedCommands.registerCommand("STOPAlgae", kAlgaeCollector.stopCommand());

    // vision alignment

    NamedCommands.registerCommand("SeesRightCoralStation", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 2 : 12;
      return kFrontTopPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesLeftCoralStation", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 1 : 13;
      return kFrontTopPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("AlignToLeftCoralStation", kSwerveDrive.alignToCoralStation(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 1 : 13;
      return kFrontTopPhotonCamera.getAprilTagById(id);
    }));
    
    NamedCommands.registerCommand("AlignToRightCoralStation", kSwerveDrive.alignToCoralStation(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 2 : 12;
      return kFrontTopPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("TurnToLeftCoralStationAngle", kSwerveDrive.rotateToAngleInPlaceCmd(Rotation2d.fromDegrees(130)));

    NamedCommands.registerCommand("SeesLeftCoralStationButLikeBetter", kFrontTopPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 1 : 13;
      return id;
    }));

    NamedCommands.registerCommand("SeesRightCoralStationButLikeBetter", kFrontTopPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 2 : 12;
      return id;
    }));
    
    NamedCommands.registerCommand("SeesF", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return kFrontLeftPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesFButLikeBetter", kFrontLeftPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return id;
    }));

    NamedCommands.registerCommand("SeesE", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return kFrontRightPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesEButLikeBetter", kFrontRightPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return id;
    }));

    NamedCommands.registerCommand("SeesK", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 6 : 19;
      return kFrontLeftPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesKButLikeBetter", kFrontLeftPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 6 : 19;
      return id;
    }));

    NamedCommands.registerCommand("SeesI", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return kFrontRightPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesIButLikeBetter", kFrontRightPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return id;
    }));

    NamedCommands.registerCommand("SeesJ", Commands.waitUntil(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return kFrontLeftPhotonCamera.getAprilTagById(id).isPresent();
    }));
    NamedCommands.registerCommand("SeesJButLikeBetter", kFrontLeftPhotonCamera.hasVisibleTarget(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return id;
    }));

    NamedCommands.registerCommand("AlignToD", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 8 : 17;
      return kFrontLeftPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToC", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 8 : 17;
      return kFrontRightPhotonCamera.getAprilTagById(id); 
    }));
    NamedCommands.registerCommand("AlignToE", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return kFrontRightPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToF", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 9 : 22;
      return kFrontLeftPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToG", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 10 : 21;
      return kFrontRightPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToH", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 10 : 21;
      return kFrontLeftPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToK", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 6 : 19;
      return kFrontRightPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToL", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 6 : 19;
      return kFrontLeftPhotonCamera.getAprilTagById(id);
    }));

    NamedCommands.registerCommand("AlignToJ", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return kFrontLeftPhotonCamera.getAprilTagById(id);
    }));
    NamedCommands.registerCommand("AlignToI", kSwerveDrive.alignToReef(() -> {
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var id = alliance == Alliance.Red ? 11 : 20;
      return kFrontRightPhotonCamera.getAprilTagById(id);
    }));
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
