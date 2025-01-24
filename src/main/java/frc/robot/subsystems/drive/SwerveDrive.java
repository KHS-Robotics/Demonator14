/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.Constants.SwerveDriveConfig;
import frc.robot.vision.LimelightHelpers;
import frc.robot.RobotContainer;

/**
 * Swerve drive style drivetrain.
 * 
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
 */
public class SwerveDrive extends SubsystemBase {
  /** Deadband for X and Y input speeds for joystick driving. */
  private static final double kDriveVelocityDeadband = 0.005;
  /** Deadband for rotational input speed for joystick driving. */
  private static final double kDriveOmegaDeadband = 0.005;

  public static double maxSpeedMetersPerSecond = 4.6;
  public static double maxAngularSpeedRadiansPerSecond = 3 * Math.PI;

  private final SwerveModule kFrontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontLeftPivotOffsetDegrees);
  private final SwerveModule kFrontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontRightPivotOffsetDegrees);
  private final SwerveModule kRearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.REAR_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearLeftPivotOffsetDegrees);
  private final SwerveModule kRearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearRightPivotOffsetDegrees);

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
   */
  private final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      SwerveDriveConfig.kFrontLeftModuleOffset,
      SwerveDriveConfig.kFrontRightModuleOffset, SwerveDriveConfig.kRearLeftModuleOffset,
      SwerveDriveConfig.kRearRightModuleOffset);

  /**
   * Standard deviations of the pose estimate (x position in meters, y position in
   * meters, and heading in radians).
   * <p>
   * Increase these numbers to trust the state estimate less.
   */
  private static final Matrix<N3, N1> kDefaultStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision pose measurement (x position in meters, y
   * position in meters, and heading in radians).
   * <p>
   * Increase these numbers to trust the vision pose measurement less.
   */
  private static final Matrix<N3, N1> kDefaultVisionMeasurementStdDevs = VecBuilder.fill(1, 1, Double.MAX_VALUE);

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
   */
  private final SwerveDrivePoseEstimator kPoseEstimator = new SwerveDrivePoseEstimator(
      kSwerveKinematics,
      getAngleForOdometry(),
      getSwerveModulePositions(),
      new Pose2d(),
      kDefaultStateStdDevs,
      kDefaultVisionMeasurementStdDevs);

  private final PIDController xPid = new PIDController(SwerveDriveConfig.DRIVE_X_P, SwerveDriveConfig.DRIVE_X_I,
      SwerveDriveConfig.DRIVE_X_D);
  private final PIDController yPid = new PIDController(SwerveDriveConfig.DRIVE_Y_P, SwerveDriveConfig.DRIVE_Y_I,
      SwerveDriveConfig.DRIVE_Y_D);
  private final PIDController thetaPid = new PIDController(SwerveDriveConfig.DRIVE_ANGLE_P,
      SwerveDriveConfig.DRIVE_ANGLE_I,
      SwerveDriveConfig.DRIVE_ANGLE_D);

  /**
   * Constructs the Swerve Drive.
   */
  public SwerveDrive() {
    // meters
    xPid.setTolerance(0.1);
    yPid.setTolerance(0.1);

    // degrees
    thetaPid.enableContinuousInput(-180.0, 180.0);
    thetaPid.setTolerance(1);

    this.configurePathPlannerAutoBuilder();

    resetPose(new Pose2d(8.5, 4.0, getAngleForOdometry()));
    RobotContainer.kFrontCamera.addProcessResultsConsumer("FrontPhotonPoseUpdater",
        (result) -> {
          // just disabled and teleop for now
          if (RobotState.isDisabled() || RobotState.isTeleop()) {
            kPoseEstimator.addVisionMeasurement(result.estimatedRobotPose.estimatedPose.toPose2d(),
                result.estimatedRobotPose.timestampSeconds, result.stdDevs);
          }
        });
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    updateOdometry();

    var pose = getPose();
    SmartDashboard.putNumber("Pose-X", pose.getX());
    SmartDashboard.putNumber("Pose-Y", pose.getY());
    SmartDashboard.putNumber("Pose-Theta", pose.getRotation().getDegrees());
    RobotContainer.kField.setRobotPose(pose);
  }

  /** https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve */
  private void configurePathPlannerAutoBuilder() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> this.setModuleStates(speeds), // Method that will drive the robot given ROBOT
                                                                  // RELATIVE
          // ChassisSpeeds. Also optionally outputs individual module
          // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_D), // Translation PID constants
              new PIDConstants(SwerveDriveConfig.DRIVE_PATHING_ROTATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_D) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception ex) {
      ex.printStackTrace();
      DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + ex.getMessage(), false);
    }
  }

  /**
   * Gets the robot's current position on the field.
   * 
   * @return the robot's position on the field from the odometry
   */
  public Pose2d getPose() {
    return kPoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the robot's pose for odometry
   * 
   * @param pose the robot's new pose
   */
  public void resetPose(Pose2d pose) {
    kPoseEstimator.resetPosition(getAngleForOdometry(), getSwerveModulePositions(), pose);
    RobotContainer.kFrontCamera.setInitialPose(pose);
  }

  /**
   * Gets the robot's current speed.
   * 
   * @return the robot's current speed
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Sets the module states based on the desired robot speed.
   * 
   * @param chassisSpeeds the desired robot speed
   */
  private void setModuleStates(final ChassisSpeeds chassisSpeeds) {
    var desiredStates = this.getOptimizedModuleStatesFromChassisSpeeds(chassisSpeeds);
    kFrontLeft.setDesiredState(desiredStates[0], true);
    kFrontRight.setDesiredState(desiredStates[1], true);
    kRearLeft.setDesiredState(desiredStates[2], true);
    kRearRight.setDesiredState(desiredStates[3], true);
  }

  /**
   * Optimizes the swerve module states given the desired robot speeds.
   * 
   * @param originalSpeeds the speeds to optimze
   * @return the optimized swerve module states
   */
  private SwerveModuleState[] getOptimizedModuleStatesFromChassisSpeeds(final ChassisSpeeds originalSpeeds) {
    // corrections + desaturating
    var optimizedChassisSpeeds = correctForDynamics(originalSpeeds);
    var optimizedModuleStates = kSwerveKinematics.toSwerveModuleStates(optimizedChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(optimizedModuleStates, maxSpeedMetersPerSecond);
    return optimizedModuleStates;
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(final ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose = new Pose2d(
        originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
        originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = log(futureRobotPose);
    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
        twistForPose.dx / LOOP_TIME_S,
        twistForPose.dy / LOOP_TIME_S,
        twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  private static Twist2d log(final Pose2d transform) {
    final double kEps = 1E-9;
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta = -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part = transform
        .getTranslation()
        .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Returns the angle of the robot as a Rotation2d as read by the navx.
   * <p>
   * Note that {@link #getPose()} should be used when trying to access the robot's
   * actual heading (rotation)
   * relative to the field. This method is only used internally to update the
   * odometry.
   *
   * @return The angle of the robot.
   * @see #getPose()
   */
  private Rotation2d getAngleForOdometry() {
    return RobotContainer.kNavx.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info. (used for teleop)
   *
   * @param vxMetersPerSecond     Speed of the robot in the x direction (forward)
   *                              in m/s.
   * @param vyMetersPerSecond     Speed of the robot in the y direction (sideways)
   *                              in m/s.
   * @param omegaRadiansPerSecond Angular rate of the robot in rad/sec.
   * @param fieldRelative         Whether the provided x and y speeds are relative
   *                              to the
   *                              field.
   */
  public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
      boolean fieldRelative) {
    if (Math.abs(omegaRadiansPerSecond) < kDriveOmegaDeadband && Math.abs(vxMetersPerSecond) < kDriveVelocityDeadband
        && Math.abs(vyMetersPerSecond) < kDriveVelocityDeadband) {
      kFrontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontLeft.getAngle())));
      kFrontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontRight.getAngle())));
      kRearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearLeft.getAngle())));
      kRearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearRight.getAngle())));
    } else {
      // get desired chassis speeds
      ChassisSpeeds desiredChassisSpeeds = fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
              getPose().getRotation())
          : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

      // set desired chassis speeds
      this.setModuleStates(desiredChassisSpeeds);
    }
  }

  /**
   * Gets the current drive speeds and pivot angles of all swerve modules in the
   * following order: FL, FR, RL, RR.
   * 
   * @return the states of the module (speeds and angles) in the following order:
   *         FL, FR, RL, RR.
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        kFrontLeft.getState(),
        kFrontRight.getState(),
        kRearLeft.getState(),
        kRearRight.getState()
    };
  }

  /**
   * Gets the current drive distance traveled and pivot angles of all swerve
   * modules in the following order: FL, FR, RL, RR.
   * 
   * @return the states of the module (speeds and angles) in the following order:
   *         FL, FR, RL, RR.
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        kFrontLeft.getPosition(),
        kFrontRight.getPosition(),
        kRearLeft.getPosition(),
        kRearRight.getPosition()
    };
  }

  /**
   * Stops all modules and resets the internal PID controllers.
   */
  public void stop() {
    kFrontRight.stop();
    kFrontLeft.stop();
    kRearRight.stop();
    kRearLeft.stop();
    resetPID();
  }

  /**
   * Sets swerve modules to be all angled - useful when trying to stay still on an
   * incline or to help prevent getting pushed by defense.
   */
  public void lock() {
    kFrontRight.setDesiredState(0, 45);
    kFrontLeft.setDesiredState(0, -45);
    kRearRight.setDesiredState(0, -45);
    kRearLeft.setDesiredState(0, 45);
  }

  /**
   * Updates the field relative position of the robot.
   * 
   * <p>
   * 
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
   */
  private void updateOdometry() {
    // always update using traction
    updateOdometryUsingTraction();

    // only update pose using vision when in disabled or teleop
    if (RobotState.isDisabled() || RobotState.isTeleop()) {
      // TODO(work in progress): Vision adjustments using AprilTags
      // ...
      // updateOdometryUsingVision();
      // updateOdometryUsingLimelightMegatag1();
      // updateOdometryUsingLimelightMegatag2();
    }
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html#updating-the-robot-pose
   */
  private void updateOdometryUsingTraction() {
    var modulePositions = getSwerveModulePositions();
    kPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngleForOdometry(), modulePositions);
  }

  /**
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
   * 
   * <p>
   * 
   * TODO: Test and determine if this is even needed at all since we have
   * Megatag2? Might be able to remove it entirely.
   */
  private void updateOdometryUsingLimelightMegatag1() {
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    boolean doRejectUpdate = false;
    if (mt1 != null && mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    if (mt1 == null || mt1.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // kPoseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds,
      // kDefaultVisionMeasurementStdDevs);

      // for debugging/testing
      // RobotContainer.kField.getObject("LL-M1").setPose(mt1.pose);
    }
  }

  /**
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
   * 
   * <p>
   * 
   * Worked very well in the lab.
   * Also worked very at very far distances!!
   * Still have to work out some kinks/set some configurations on LL but overall
   * not too bad.
   * 
   * <p>
   * 
   * TODO: Work to make accurate+reliable. May not be able to know for sure until
   * we get the robot on the test field.
   */
  private void updateOdometryUsingLimelightMegatag2() {
    LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = false;
    // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    if (Math.abs(RobotContainer.kNavx.getRate()) > 720) {
      doRejectUpdate = true;
    }
    if (mt2 == null || mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // kPoseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
      // kDefaultVisionMeasurementStdDevs);

      // for debugging/testing
      // RobotContainer.kField.getObject("LL-M2").setPose(mt2.pose);
    }
  }

  /**
   * Holds the desired angle while translating.
   * 
   * @param xSpeed        x speed in m/s
   * @param ySpeed        y speed in m/s
   * @param setpointAngle the desired robot angle to hold about the robot's center
   * @param fieldOriented field relative speeds (true) or robot relative speeds
   *                      (false)
   */
  public void holdAngleWhileDriving(double xSpeed, double ySpeed, Rotation2d setpointAngle, boolean fieldOriented) {
    var rotateOutput = MathUtil
        .clamp(thetaPid.calculate(getPose().getRotation().getDegrees(), normalizeAngle(setpointAngle.getDegrees())), -1,
            1)
        * maxAngularSpeedRadiansPerSecond;
    this.drive(xSpeed, ySpeed, rotateOutput, fieldOriented);
  }

  /**
   * Rotate to a desired angle about the robot's center.
   * 
   * @param setpointAngle
   */
  public void rotateToAngleInPlace(Rotation2d setpointAngle) {
    this.holdAngleWhileDriving(0, 0, setpointAngle, false);
  }

  /**
   * Sets the drive to go to the desired pose given the robot's current pose vs
   * target
   * 
   * @param target        the desired pose the robot should go to
   * @param fieldOriented field relative speeds (true) or robot relative speeds
   *                      (false)
   */
  public void goToPose(Pose2d target, boolean fieldOriented) {
    Pose2d pose = getPose();
    double xSpeed = MathUtil.clamp(xPid.calculate(pose.getX(), target.getX()), -1, 1) * maxSpeedMetersPerSecond;
    double ySpeed = MathUtil.clamp(yPid.calculate(pose.getY(), target.getY()), -1, 1) * maxSpeedMetersPerSecond;
    double vTheta = MathUtil.clamp(thetaPid.calculate(normalizeAngle(pose.getRotation().getDegrees()),
        normalizeAngle(target.getRotation().getDegrees())), -1, 1) * maxAngularSpeedRadiansPerSecond;
    this.drive(xSpeed, ySpeed, vTheta, fieldOriented);
  }

  /**
   * Gets if the robot's heading is at the desired angle.
   * 
   * @return true if the robot's heading is at the desired angle, false otherwise
   */
  public boolean atAngleSetpoint() {
    return thetaPid.atSetpoint();
  }

  /**
   * Resets the internal X, Y and Theta PID controllers.
   */
  public void resetPID() {
    xPid.reset();
    yPid.reset();
    thetaPid.reset();
  }

  /**
   * Clamps an angle to be from [-180, 180]
   * 
   * @param angle the angle to clamp
   * @return the newly clamped angle from [-180, 180]
   */
  private static double normalizeAngle(double angle) {
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
}
