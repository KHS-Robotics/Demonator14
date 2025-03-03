package frc.robot.subsystems.cameras;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.cameras.CameraConfig.PhotonVisionConfig;

public class AprilTagTarget {
  public final Pose3d position;
  public final PhotonTrackedTarget tag;
  public final int id;

  public AprilTagTarget(Pose3d position, PhotonTrackedTarget tag, int id) {
    this.position = position;
    this.tag = tag;
    this.id = id;
  }

  public double getDifferenceX() {
    var dist = tag.getBestCameraToTarget();
    return dist.getX();
  }

  public double getDifferenceY() {
    var dist = tag.getBestCameraToTarget();
    return dist.getY();
  }

  public double getTargetAngle() {
    var targetPositionOpt = PhotonVisionConfig.kTagLayout.getTagPose(tag.getFiducialId());
    if (targetPositionOpt.isEmpty())
      return 0;

    return normalizeAngle(Math.toDegrees(targetPositionOpt.get().getRotation().getAngle()) + 180);
  }

  @Override
  public String toString() {
    return "X = " + getDifferenceX() + " :: Y = " + getDifferenceY() + " :: Theta = " + getTargetAngle();
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
