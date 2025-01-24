// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cameras;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PhotonVisionConfig;

/**
 * Encapsulates an April Tag specific camera using Photon Vision.
 * 
 * <p>
 * 
 * https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/robot-pose-estimator.html
 */
public class PhotonAprilTagCamera extends SubsystemBase {
  private Optional<PhotonVisionUpdate> latestUpdate = Optional.empty();
  private HashMap<String, Consumer<PhotonVisionUpdate>> photonUpdateConsumers = new HashMap<>();

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new camera to track April Tags using Photon Vision.
   * 
   * @param cameraName   The nickname of the camera (found in the
   *                     PhotonVision UI)
   * @param cameraOffset Transform3d from the center of the robot to the
   *                     camera mount position (ie, robot ➔ camera) in the
   *                     <a href=
   *                     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *                     Coordinate System</a>
   */
  public PhotonAprilTagCamera(String cameraName, Transform3d cameraOffset) {
    setName(cameraName);

    this.camera = new PhotonCamera(cameraName);

    this.poseEstimator = new PhotonPoseEstimator(PhotonVisionConfig.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
    this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    // make sure to call once per loop to get consistent results
    processResults();
  }

  /**
   * Adds a listener for the processed results from photon vision.
   * 
   * @param consumerName         the name of this consumer to add
   * @param photonUpdateConsumer the action on the processed update from photon
   *                             vision
   */
  public void addProcessResultsConsumer(String consumerName, Consumer<PhotonVisionUpdate> photonUpdateConsumer) {
    this.photonUpdateConsumers.put(consumerName, photonUpdateConsumer);
  }

  /**
   * Removes a listener for the processed results from photon vision.
   * 
   * @param consumerName the name of this consumer to remove
   */
  public void removeProcessResultsConsumer(String consumerName) {
    this.photonUpdateConsumers.remove(consumerName);
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
   * Gets the latest processed update from photon vision from
   * {@link #processResults()}.
   * 
   * @return the latest processed update from photon vision
   */
  public Optional<PhotonVisionUpdate> getLatestUpdate() {
    return latestUpdate;
  }

  /**
   * Processes all current unread results and sends the results to the consumers.
   */
  private void processResults() {
    // get unprocessed results from photon
    var unprocessedCameraResults = camera.getAllUnreadResults();
    for (var cameraResult : unprocessedCameraResults) {
      // process results from photon
      var photonPoseUpdate = poseEstimator.update(cameraResult);

      // check if processed result has an update from photon
      if (photonPoseUpdate.isPresent()) {
        // get the update from photon
        var photonPose = photonPoseUpdate.get();

        // calculate standard deviations and set the latest update
        var stdDevs = getEstimatedStdDevs(cameraResult, photonPose.estimatedPose.toPose2d());
        latestUpdate = Optional.of(new PhotonVisionUpdate(cameraResult, photonPose, stdDevs));

        // notify consumers with the latest update
        photonUpdateConsumers.values().stream().forEach((c) -> c.accept(latestUpdate.get()));
      } else {
        // no update from photon
        latestUpdate = Optional.empty();
      }
    }
  }

  /**
   * Calcualtes the estimated standard deviations based on number of visible April
   * Tags and their distances from the camera.
   * 
   * @param result        the result from photon
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getEstimatedStdDevs(PhotonPipelineResult result, Pose2d estimatedPose) {
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
   * Holds all applicable information to a photon vision update.
   */
  public static class PhotonVisionUpdate {
    public final PhotonPipelineResult cameraResult;
    public final EstimatedRobotPose estimatedRobotPose;
    public final Matrix<N3, N1> stdDevs;

    /**
     * Holds all applicable information to a photon vision update.
     * 
     * @param cameraResult       the pipeline result from
     *                           {@link org.photonvision.PhotonCamera#getAllUnreadResults()}
     * @param estimatedRobotPose the estimated robot pose from
     *                           {@link org.photonvision.PhotonPoseEstimator#update(PhotonPipelineResult)}
     * @param stdDevs            the standard deviations to use for the consumer to use
     */
    public PhotonVisionUpdate(PhotonPipelineResult cameraResult, EstimatedRobotPose estimatedRobotPose,
        Matrix<N3, N1> stdDevs) {
      this.cameraResult = cameraResult;
      this.estimatedRobotPose = estimatedRobotPose;
      this.stdDevs = stdDevs;
    }
  }
}
