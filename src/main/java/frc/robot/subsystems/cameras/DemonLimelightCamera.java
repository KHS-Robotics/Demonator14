// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cameras;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

/**
 * Encapsulates a camera using Limelight.
 * 
 * <p>
 * 
 * https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary
 */
public class DemonLimelightCamera extends SubsystemBase {
  /** Enum to hold the possible limelight pose estimate algorithms. */
  public static enum LimelightPoseEstimateAlgorithm {
    /**
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
     */
    kMegatag1,
    /**
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
     */
    kMegatag2;
  }

  private Optional<PoseEstimate> latestPoseEstimate = Optional.empty();
  private LimelightPoseEstimateAlgorithm currentPEAlgorithm = LimelightPoseEstimateAlgorithm.kMegatag2;
  private boolean enableAprilTagUpdates = true;

  private final String limelightName;
  private final Supplier<Pose2d> currentPose;
  private final Supplier<Double> currentGyroRate;

  /**
   * Creates a new Limelight camera for pose estimation using AprilTags.
   * 
   * @param limelightName   the name of the configured limelight from the UI
   * @param peAlgorithm     the algorithm to use for AprilTags
   * @param currentPose     the current pose of the robot
   * @param currentGyroRate the current rate of the gyro
   */
  public DemonLimelightCamera(String limelightName, LimelightPoseEstimateAlgorithm peAlgorithm,
      Supplier<Pose2d> currentPose, Supplier<Double> currentGyroRate) {
    this.limelightName = limelightName;
    this.currentPose = currentPose;
    this.currentGyroRate = currentGyroRate;

    setName(limelightName);
    setPoseEstimateAlgorithm(peAlgorithm);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    updateLatestPoseEstimate();
    putLatestTelemetryToSmartDashboard();
  }

  /**
   * Set the pose estimate algorithm to use.
   * 
   * @param peAlgorithm the pose estimate algorithm to use
   * @see {@link LimelightPoseEstimateAlgorithm}
   */
  public void setPoseEstimateAlgorithm(LimelightPoseEstimateAlgorithm peAlgorithm) {
    this.currentPEAlgorithm = peAlgorithm;
  }

  /**
   * Gets the pose estimate algorithm that is currently being used
   * 
   * @return the pose estimate algorithm that is currently being used
   */
  public LimelightPoseEstimateAlgorithm getCurrentPoseEstimateAlgorithm() {
    return this.currentPEAlgorithm;
  }

  /**
   * Gets the latest pose estimate from Limelight.
   * 
   * @return the latest pose estimate from Limelight
   */
  public Optional<PoseEstimate> getLatestPoseEstimate() {
    return latestPoseEstimate;
  }

  /**
   * Sets the camera to receive AprilTag updates or not for the odometry.
   * 
   * @param enableAprilTagUpdates true to enable AprilTag updates, false to disable AprilTag updates
   */
  public void setEnableAprilTagUpdates(boolean enableAprilTagUpdates) {
    this.enableAprilTagUpdates = enableAprilTagUpdates;
  }

  /**
   * Updates the latest pose estimate from the Limelight.
   */
  private void updateLatestPoseEstimate() {
    // check if updates are enabled
    if (!enableAprilTagUpdates) {
      latestPoseEstimate = Optional.empty();
      return;
    }

    // process based on selected algorithm
    switch (currentPEAlgorithm) {
      case kMegatag1:
        updateLatestMegatag1PoseEstimate();
        break;
      case kMegatag2:
        updateLatestMegatag2PoseEstimate();
        break;
      default:
        DriverStation.reportWarning("Invalid limelight algorithm.", false);
        break;
    }
  }

  /**
   * Updates the latest pose estimate from the Limelight using Megatag v1.
   */
  private void updateLatestMegatag1PoseEstimate() {
    // Megatag1
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    boolean doRejectUpdate = false;
    // if only one tag is visible, perform stricter checks
    if (mt1 != null && mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      // reject if too ambiguous
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      // reject if distance is too far
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    // reject if there are no tags visible
    if (mt1 == null || mt1.tagCount == 0) {
      doRejectUpdate = true;
    }

    // set latest update
    latestPoseEstimate = !doRejectUpdate ? Optional.of(mt1) : Optional.empty();
  }

  /**
   * Updates the latest pose estimate from the Limelight using Megatag v2.
   */
  private void updateLatestMegatag2PoseEstimate() {
    // Megatag2
    LimelightHelpers.SetRobotOrientation(limelightName, currentPose.get().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    boolean doRejectUpdate = false;
    // reject if our angular velocity is greater than 720 degrees per second
    if (Math.abs(currentGyroRate.get()) > 720) {
      doRejectUpdate = true;
    }
    // reject if there are no tags visible
    if (mt2 == null || mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    // set latest update
    latestPoseEstimate = !doRejectUpdate ? Optional.of(mt2) : Optional.empty();
  }

  /**
   * Updates the telemetry from the camera on SmartDashboard.
   */
  private void putLatestTelemetryToSmartDashboard() {
    SmartDashboard.putBoolean(getName() + "-HasPoseEstimate", latestPoseEstimate.isPresent());
    SmartDashboard.putString(getName() + "-Algorithm", currentPEAlgorithm.toString());
    SmartDashboard.putBoolean(getName() + "-Enabled", enableAprilTagUpdates);

    if (latestPoseEstimate.isPresent()) {
      SmartDashboard.putNumber(getName() + "NumAprilTags", latestPoseEstimate.get().tagCount);
    }
  }
}
