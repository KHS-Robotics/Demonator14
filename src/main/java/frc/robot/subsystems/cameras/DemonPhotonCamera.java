// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PhotonVisionConfig;

/**
 * Encapsulates a camera using Photon Vision.
 * 
 * <p>
 * 
 * <a href=
 * "https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/robot-pose-estimator.html">
 * AprilTags
 * </a>
 * 
 * <p>
 * 
 * <a href=
 * "https://docs.photonvision.org/en/v2025.1.1/docs/reflectiveAndShape/thresholding.html">
 * Algae
 * </a>
 */
public class DemonPhotonCamera extends SubsystemBase {
  private PhotonPipelineMode currentPipelineMode;
  private Optional<PhotonPoseUpdate> aprilTagUpdate = Optional.empty();
  private Optional<List<PhotonTrackedTarget>> algaeTargets = Optional.empty();
  private Optional<PhotonTrackedTarget> bestAlgaeTarget = Optional.empty();
  private boolean enableAprilTagUpdates = true;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new camera to track AprilTags using Photon Vision.
   * 
   * @param cameraName   The nickname of the camera (found in the
   *                     PhotonVision UI)
   * @param cameraOffset Transform3d from the center of the robot to the
   *                     camera mount position (ie, robot ➔ camera) in the
   *                     <a href=
   *                     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *                     Coordinate System</a>
   */
  public DemonPhotonCamera(String cameraName, Transform3d cameraOffset) {
    setName(cameraName);

    camera = new PhotonCamera(cameraName);

    poseEstimator = new PhotonPoseEstimator(PhotonVisionConfig.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    setAprilTagMode();
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    // make sure to call once per loop to get consistent results
    updateLatestVisionResults();
    putLatestTelemetryToSmartDashboard();
  }

  /**
   * Sets the camera pipeline to AprilTag mode.
   */
  public void setAprilTagMode() {
    if (currentPipelineMode == PhotonPipelineMode.kAprilTags)
      return;

    algaeTargets = Optional.empty();
    bestAlgaeTarget = Optional.empty();
    currentPipelineMode = PhotonPipelineMode.kAprilTags;
    camera.setPipelineIndex(currentPipelineMode.index);
  }

  /**
   * Sets the camera pipeline to Algae mode.
   */
  public void setAlgaeMode() {
    if (currentPipelineMode == PhotonPipelineMode.kAlgae)
      return;

    aprilTagUpdate = Optional.empty();
    currentPipelineMode = PhotonPipelineMode.kAlgae;
    camera.setPipelineIndex(currentPipelineMode.index);
  }

  /**
   * Sets the initial pose for
   * {@link org.photonvision.PhotonPoseEstimator#setReferencePose(Pose2d)} and
   * {@link org.photonvision.PhotonPoseEstimator#setLastPose(Pose2d)}
   * 
   * @param pose the initial pose for the photon pose estimator
   */
  public void setInitialPose(Pose2d pose) {
    poseEstimator.setReferencePose(pose);
    poseEstimator.setLastPose(pose);
  }

  /**
   * Gets the latest processed update from photon vision for AprilTags.
   * 
   * @return the latest processed update from photon vision for AprilTags
   */
  public Optional<PhotonPoseUpdate> getLatestAprilTagResults() {
    if (currentPipelineMode != PhotonPipelineMode.kAprilTags) {
      DriverStation.reportWarning("Attempting to get AprilTags results in Algae mode!", false);
    }

    return aprilTagUpdate;
  }

  /**
   * Gets the latest processed update from photon vision from
   * {@link org.photonvision.targeting.PhotonPipelineResult#getTargets()}.
   * 
   * @return all targets from the latest pipeline result
   */
  public Optional<List<PhotonTrackedTarget>> getLatestAlgaeTargets() {
    if (currentPipelineMode != PhotonPipelineMode.kAlgae) {
      DriverStation.reportWarning("Attempting to get Algae results in AprilTag mode!", false);
    }

    return algaeTargets;
  }

  /**
   * Gets the latest processed update from photon vision from
   * {@link org.photonvision.targeting.PhotonPipelineResult#getBestTarget()}.
   * 
   * @return the best target from the latest pipeline result
   */
  public Optional<PhotonTrackedTarget> getLatestBestAlgaeTarget() {
    if (currentPipelineMode != PhotonPipelineMode.kAlgae) {
      DriverStation.reportWarning("Attempting to get Algae results in AprilTag mode!", false);
    }

    return bestAlgaeTarget;
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
   * Processes all current unread results.
   */
  private void updateLatestVisionResults() {
    // get unprocessed results from photon
    var unprocessedCameraResults = camera.getAllUnreadResults();

    for (var cameraResult : unprocessedCameraResults) {
      // process based on selected pipeline mode
      switch (currentPipelineMode) {
        case kAprilTags:
          processAprilTagResult(cameraResult);
          break;
        case kAlgae:
          processAlgaeResult(cameraResult);
          break;
        default:
          DriverStation.reportWarning("Invalid pipeline mode.", false);
      }
    }
  }

  /**
   * Processes the camera result for the AprilTag pipeline.
   * 
   * @param cameraResult the camera result from Photon
   */
  private void processAprilTagResult(PhotonPipelineResult cameraResult) {
    // check if AprilTag updates are enabled
    if (!enableAprilTagUpdates) {
      aprilTagUpdate = Optional.empty();
      return;
    }

    // process results from photon
    var photonPoseUpdate = poseEstimator.update(cameraResult);

    // check if processed result has an update from photon
    if (photonPoseUpdate.isPresent()) {
      // get the update from photon
      var photonPose = photonPoseUpdate.get();

      // calculate standard deviations and set the latest update
      var stdDevs = getEstimatedStdDevsForAprilTagResult(cameraResult, photonPose.estimatedPose.toPose2d());
      aprilTagUpdate = Optional.of(new PhotonPoseUpdate(cameraResult, photonPose, stdDevs));
    } else {
      // no update from photon
      aprilTagUpdate = Optional.empty();
    }
  }

  /**
   * Processes the camera result for the Algae pipeline.
   * 
   * @param cameraResult the camera result from Photon
   */
  private void processAlgaeResult(PhotonPipelineResult cameraResult) {
    // check if any visible targets
    if (cameraResult.hasTargets()) {
      algaeTargets = Optional.of(cameraResult.getTargets());
      bestAlgaeTarget = Optional.of(cameraResult.getBestTarget());
    } else {
      algaeTargets = Optional.empty();
      bestAlgaeTarget = Optional.empty();
    }
  }

  /**
   * Calcualtes the estimated standard deviations based on number of visible April
   * Tags and their distances from the camera.
   * 
   * @param result        the result from photon
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getEstimatedStdDevsForAprilTagResult(PhotonPipelineResult result, Pose2d estimatedPose) {
    try {
      if (result == null) {
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      }

      var estStdDevs = VecBuilder.fill(0.5, 0.5, 999999999);
      var targets = result.getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }

      if (numTags == 0)
        return estStdDevs;

      avgDist /= numTags;

      // Decrease std devs if multiple targets are visible
      if (numTags > 1 && avgDist < 3.5)
        estStdDevs = VecBuilder.fill(0.2, 0.2, 999999999);

      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 3.5)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
    } catch (Exception exception) {
      exception.printStackTrace();
      DriverStation.reportError("Failed to calculate stddev for Photon.", false);
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
  }

  /**
   * Updates the telemetry from the camera on SmartDashboard.
   */
  private void putLatestTelemetryToSmartDashboard() {
    SmartDashboard.putBoolean(getName() + "-Enabled", enableAprilTagUpdates);
    SmartDashboard.putBoolean(getName() + "-HasAprilTagUpdate", aprilTagUpdate.isPresent());
    SmartDashboard.putBoolean(getName() + "-HasAlgaeTargets", algaeTargets.isPresent());
    SmartDashboard.putBoolean(getName() + "-HasBestAlgaeTarget", bestAlgaeTarget.isPresent());

    if (aprilTagUpdate.isPresent()) {
      SmartDashboard.putNumber(getName() + "NumAprilTags", aprilTagUpdate.get().cameraResult.getTargets().size());
    }
    if (algaeTargets.isPresent()) {
      SmartDashboard.putNumber(getName() + "NumAlgae", algaeTargets.get().size());
    }
  }

  /**
   * Enumeration for Photon pipeline modes for the camera.
   */
  private static enum PhotonPipelineMode {
    /** AprilTag detection mode. */
    kAprilTags(0),
    /** Algae detection mode. */
    kAlgae(1);

    /** The Photon pipeline index. */
    public final int index;

    /**
     * Enumeration for Photon pipeline modes for the camera.
     * 
     * @param index the Photon pipeline index
     */
    PhotonPipelineMode(int index) {
      this.index = index;
    }
  }

  /**
   * Holds all applicable information to a photon vision update.
   */
  public static class PhotonPoseUpdate {
    /** The camera result used for the update. */
    public final PhotonPipelineResult cameraResult;
    /** The estimated robot pose from the pose estimator. */
    public final EstimatedRobotPose estimatedRobotPose;
    /** The standard deviations to use for the vision update. */
    public final Matrix<N3, N1> stdDevs;

    /**
     * Holds all applicable information to a photon vision update.
     * 
     * @param cameraResult       the pipeline result from
     *                           {@link org.photonvision.PhotonCamera#getAllUnreadResults()}
     * @param estimatedRobotPose the estimated robot pose from
     *                           {@link org.photonvision.PhotonPoseEstimator#update(PhotonPipelineResult)}
     * @param stdDevs            the standard deviations for the vision update
     */
    private PhotonPoseUpdate(PhotonPipelineResult cameraResult, EstimatedRobotPose estimatedRobotPose,
        Matrix<N3, N1> stdDevs) {
      this.cameraResult = cameraResult;
      this.estimatedRobotPose = estimatedRobotPose;
      this.stdDevs = stdDevs;
    }
  }
}
