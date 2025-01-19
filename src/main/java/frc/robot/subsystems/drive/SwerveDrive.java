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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.vision.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static double maxSpeedMetersPerSecond = 4.6;
  public static double maxAngularSpeedRadiansPerSecond = 3 * Math.PI;

  private final PIDController anglePid;
  private final PIDController xPid;
  private final PIDController yPid;

  public static final SwerveModule frontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      Constants.DRIVE_MODULE_PIVOT_P,
      Constants.DRIVE_MODULE_PIVOT_I,
      Constants.DRIVE_MODULE_PIVOT_D,
      Constants.DRIVE_MODULE_P,
      Constants.DRIVE_MODULE_I,
      Constants.DRIVE_MODULE_D,
      Constants.DRIVE_MODULE_KS,
      Constants.DRIVE_MODULE_KV,
      Constants.DRIVE_MODULE_KA,
      RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      Constants.FRONT_LEFT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule frontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      Constants.DRIVE_MODULE_PIVOT_P,
      Constants.DRIVE_MODULE_PIVOT_I,
      Constants.DRIVE_MODULE_PIVOT_D,
      Constants.DRIVE_MODULE_P,
      Constants.DRIVE_MODULE_I,
      Constants.DRIVE_MODULE_D,
      Constants.DRIVE_MODULE_KS,
      Constants.DRIVE_MODULE_KV,
      Constants.DRIVE_MODULE_KA,
      RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      Constants.FRONT_RIGHT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule rearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      Constants.DRIVE_MODULE_PIVOT_P,
      Constants.DRIVE_MODULE_PIVOT_I,
      Constants.DRIVE_MODULE_PIVOT_D,
      Constants.DRIVE_MODULE_P,
      Constants.DRIVE_MODULE_I,
      Constants.DRIVE_MODULE_D,
      Constants.DRIVE_MODULE_KS,
      Constants.DRIVE_MODULE_KV,
      Constants.DRIVE_MODULE_KA,
      RobotMap.REAR_LEFT_PIVOT_ENCODER,
      Constants.REAR_LEFT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule rearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      Constants.DRIVE_MODULE_PIVOT_P,
      Constants.DRIVE_MODULE_PIVOT_I,
      Constants.DRIVE_MODULE_PIVOT_D,
      Constants.DRIVE_MODULE_P,
      Constants.DRIVE_MODULE_I,
      Constants.DRIVE_MODULE_D,
      Constants.DRIVE_MODULE_KS,
      Constants.DRIVE_MODULE_KV,
      Constants.DRIVE_MODULE_KA,
      RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      Constants.REAR_RIGHT_PIVOT_OFFSET_DEGREES);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.FRONT_LEFT_OFFSET,
      Constants.FRONT_RIGHT_OFFSET, Constants.REAR_LEFT_OFFSET, Constants.REAR_RIGHT_OFFSET);

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      RobotContainer.kNavx.getRotation2d(),
      new SwerveModulePosition[] {
          new SwerveModulePosition(0, Rotation2d.fromDegrees(frontLeft.getAngle())),
          new SwerveModulePosition(0, Rotation2d.fromDegrees(frontRight.getAngle())),
          new SwerveModulePosition(0, Rotation2d.fromDegrees(rearLeft.getAngle())),
          new SwerveModulePosition(0, Rotation2d.fromDegrees(rearRight.getAngle()))
      },
      new Pose2d(8.5, 4.0, Rotation2d.fromDegrees(0)),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(6, 6, Double.MAX_VALUE));

  /**
   * Constructs the Swerve Drive.
   */
  public SwerveDrive() {
    // meters
    xPid = new PIDController(Constants.DRIVE_X_P, Constants.DRIVE_X_I, Constants.DRIVE_X_D);
    xPid.setTolerance(0.1);
    yPid = new PIDController(Constants.DRIVE_Y_P, Constants.DRIVE_Y_I, Constants.DRIVE_Y_D);
    yPid.setTolerance(0.1);

    // degrees
    anglePid = new PIDController(Constants.DRIVE_ANGLE_P, Constants.DRIVE_ANGLE_I, Constants.DRIVE_ANGLE_D);
    anglePid.enableContinuousInput(-180.0, 180.0);
    anglePid.setTolerance(1);

    this.configurePathPlannerAutoBuilder();
  }

  /** https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve */
  private void configurePathPlannerAutoBuilder() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      if (!config.hasValidConfig()) {
        DriverStation.reportError("Invalid RobotConfig for PathPlanner. See NetworkTables Alerts for more details.", false);
      }

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
              new PIDConstants(Constants.DRIVE_PATHING_TRANSLATION_P, Constants.DRIVE_PATHING_TRANSLATION_I,
                  Constants.DRIVE_PATHING_TRANSLATION_D), // Translation PID constants
              new PIDConstants(Constants.DRIVE_PATHING_ROTATION_P, Constants.DRIVE_PATHING_ROTATION_I,
                  Constants.DRIVE_PATHING_ROTATION_D) // Rotation PID constants
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
   * Returns the angle of the robot as a Rotation2d as read by the navx.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((-RobotContainer.kNavx.getAngle()) % 360);
  }

  /**
   * Method to drive the robot using joystick info. (used for teleop)
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in m/s.
   * @param rot           Angular rate of the robot in rad/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (Math.abs(rot) < 0.005 && Math.abs(xSpeed) < 0.005 && Math.abs(ySpeed) < 0.005) {
      frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontLeft.getAngle())));
      frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontRight.getAngle())));
      rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearLeft.getAngle())));
      rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearRight.getAngle())));
    } else {
      // get desired chassis speeds
      ChassisSpeeds desiredChassisSpeeds = fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
          : new ChassisSpeeds(xSpeed, ySpeed, rot);

      // corrections + desaturating
      desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
      var desiredModuleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeedMetersPerSecond);

      // set calculated states
      this.setModuleStates(desiredModuleStates);
    }
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0], true);
    frontRight.setDesiredState(desiredStates[1], true);
    rearLeft.setDesiredState(desiredStates[2], true);
    rearRight.setDesiredState(desiredStates[3], true);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    var desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getAngle());
    var desiredModuleStates = kinematics.toSwerveModuleStates(desiredChassisSpeed);
    this.setModuleStates(desiredModuleStates);
  }

  public void holdAngleWhileDriving(double x, double y, Rotation2d setAngle, boolean fieldOriented) {
    var rotateOutput = MathUtil
        .clamp(anglePid.calculate(getPose().getRotation().getDegrees(), normalizeAngle(setAngle.getDegrees())), -1, 1)
        * maxAngularSpeedRadiansPerSecond;
    this.drive(x, y, rotateOutput, fieldOriented);
  }

  public void rotateToAngleInPlace(double setAngle) {
    this.holdAngleWhileDriving(0, 0, Rotation2d.fromDegrees(setAngle), false);
  }

  public void goToPose(Pose2d target, boolean fieldOriented) {
    Pose2d pose = getPose();
    double xSpeed = MathUtil.clamp(xPid.calculate(pose.getX(), target.getX()), -1, 1) * maxSpeedMetersPerSecond;
    double ySpeed = MathUtil.clamp(yPid.calculate(pose.getY(), target.getY()), -1, 1) * maxSpeedMetersPerSecond;
    double vTheta = MathUtil.clamp(anglePid.calculate(normalizeAngle(pose.getRotation().getDegrees()),
        normalizeAngle(target.getRotation().getDegrees())), -1, 1) * maxAngularSpeedRadiansPerSecond;
    this.drive(xSpeed, ySpeed, vTheta, fieldOriented);
  }

  /**
   * Sets swerve modules to be all angled - useful when trying to stay still on an
   * incline or to help prevent getting pushed by defense.
   */
  public void lock() {
    frontRight.setDesiredState(0, 45);
    frontLeft.setDesiredState(0, -45);
    rearRight.setDesiredState(0, -45);
    rearLeft.setDesiredState(0, 45);
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }

  public void stop() {
    frontRight.stop();
    frontLeft.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public boolean atAngleSetpoint() {
    return anglePid.atSetpoint();
  }

  public void resetPid() {
    xPid.reset();
    yPid.reset();
    anglePid.reset();
  }

  public Pose2d getPoseInverted() {
    return new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation().plus(Rotation2d.fromDegrees(180)));
  }

  /**
   * Updates the field relative position of the robot.
   */
  private void updateOdometry() {
    updateOdometryUsingTraction();

    // TODO(work in progress): Vision adjustments using AprilTags
    // ...
    // updateOdometryUsingVision();
  }

  private void updateOdometryUsingTraction() {
    var modulePositions = getSwerveModulePositions();
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngle(), modulePositions);
  }

  /**
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
   */
  private void updateOdometryUsingLimelightMegatag1() {
    boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      // poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      RobotContainer.kField.getObject("LL-Megatag1").setPose(mt1.pose);
    }
  }

  /**
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
   */
  private void updateOdometryUsingLimelightMegatag2() {
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (Math.abs(RobotContainer.kNavx.getRate()) > 720) // if our angular velocity is greater than 720 degrees per
                                                        // second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      // poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      RobotContainer.kField.getObject("LL-Megatag2").setPose(mt2.pose);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateOdometryUsingLimelightMegatag1();
    updateOdometryUsingLimelightMegatag2();

    var pose = getPose();
    RobotContainer.kField.setRobotPose(pose);
  }

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
