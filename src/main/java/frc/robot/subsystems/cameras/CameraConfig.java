package frc.robot.subsystems.cameras;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.cameras.DemonLimelightCamera.LimelightPoseEstimateAlgorithm;

public final class CameraConfig {
  // red reef
  public static final ArrayList<Integer> kRedAllianceReefFiducialIds = new ArrayList<>();

  // blue reef
  public static final ArrayList<Integer> kBlueAllianceReefFiducialIds = new ArrayList<>();
  // red coral station
  public static final ArrayList<Integer> kRedAllianceCoralFiducialIds = new ArrayList<>();
  // blue coral station
  public static final ArrayList<Integer> kBlueAllianceCoralFiducialIds = new ArrayList<>();

  static {
    kRedAllianceReefFiducialIds.add(6);
    kRedAllianceReefFiducialIds.add(7);
    kRedAllianceReefFiducialIds.add(8);
    kRedAllianceReefFiducialIds.add(9);
    kRedAllianceReefFiducialIds.add(10);
    kRedAllianceReefFiducialIds.add(11);

    kBlueAllianceReefFiducialIds.add(17);
    kBlueAllianceReefFiducialIds.add(18);
    kBlueAllianceReefFiducialIds.add(19);
    kBlueAllianceReefFiducialIds.add(20);
    kBlueAllianceReefFiducialIds.add(21);
    kBlueAllianceReefFiducialIds.add(22);

    kRedAllianceCoralFiducialIds.add(1);
    kRedAllianceCoralFiducialIds.add(2);

    kBlueAllianceCoralFiducialIds.add(12);
    kBlueAllianceCoralFiducialIds.add(13);
  }

  public class PhotonVisionConfig {
    /** The nickname of the camera (found in the PhotonVision UI). */
    public static final String kFrontRightCameraName = "Right";
    public static final String kFrontLeftCameraName = "Left";
    public static final String kFrontTopCameraName = "Top";
    /**
     * Transform3d from the center of the robot to the camera mount position (ie,
     * robot ➔ camera) in the <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     * Coordinate System</a>.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-a-photonposeestimator
     */
    public static final Transform3d kRobotToFrontRightCamera = new Transform3d(Units.inchesToMeters(10.5),
        Units.inchesToMeters(-7), Units.inchesToMeters(16.5), new Rotation3d(0, Math.toRadians(20), 0));
    public static final Transform3d kRobotToFrontLeftCamera = new Transform3d(Units.inchesToMeters(10.5),
        Units.inchesToMeters(7), Units.inchesToMeters(16.5), new Rotation3d(0, Math.toRadians(20), 0));
    public static final Transform3d kRobotToFrontTopCamera = new Transform3d(Units.inchesToMeters(10.5),
        Units.inchesToMeters(7), Units.inchesToMeters(40.125), new Rotation3d(0, Math.toRadians(20), 0));
    /**
     * The layout of the AprilTags on the field.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-an-apriltagfieldlayout
     * <p>
     * https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
     */
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);
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
    public static final Transform3d kRobotToRearCamera = new Transform3d(Units.inchesToMeters(-10),
        Units.inchesToMeters(6), Units.inchesToMeters(30), new Rotation3d(0, Math.toRadians(6), Math.toRadians(180)));
  }
}
