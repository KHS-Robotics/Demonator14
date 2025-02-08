package frc.robot.subsystems.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.cameras.DemonLimelightCamera.LimelightPoseEstimateAlgorithm;

public class CameraConfig {
  public class PhotonVisionConfig {
    /** The nickname of the camera (found in the PhotonVision UI). */
    public static final String kLowerFrontCameraName = "4342_AprilTag_1";
    /**
     * Transform3d from the center of the robot to the camera mount position (ie,
     * robot ➔ camera) in the <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     * Coordinate System</a>.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-a-photonposeestimator
     */
    public static final Transform3d kRobotToLowerFrontCamera = new Transform3d(Units.inchesToMeters(7.6882),
        Units.inchesToMeters(13.0), Units.inchesToMeters(12.1545), new Rotation3d(0, Math.toRadians(-60), 0));
    /**
     * The layout of the AprilTags on the field.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-an-apriltagfieldlayout
     * <p>
     * https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
     */
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public class LimelightConfig {
    /** The name of the camera from the UI. */
    public static final String kRearCameraName = "limelight";
    /** The pose estimation algorithm to use, */
    public static final LimelightPoseEstimateAlgorithm kPoseAlgorithm = LimelightPoseEstimateAlgorithm.Megatag2;
    /**
     * Transform3d from the center of the robot to the camera mount position (ie,
     * robot ➔ camera) in the <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     * Coordinate System</a>.
     * <p>
     * <b>This must be configured in the Limelight UI too under 3-D.</b>
     */
    public static final Transform3d kRobotToRearCamera = new Transform3d(Units.inchesToMeters(0),
        Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation3d(0, 0, 0));
  }
}
