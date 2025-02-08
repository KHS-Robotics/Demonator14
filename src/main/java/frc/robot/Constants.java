package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.cameras.DemonLimelightCamera.LimelightPoseEstimateAlgorithm;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  /**
   * Configurations for PhotonVision.
   */
  public static final class PhotonVisionConfig {
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

  /**
   * Configurations for Limelight.
   */
  public static final class LimelightConfig {
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

  public static final class ElevatorConfig {
    public static final double kElevatorP = 0.0;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0; 
    /** Gravity term in volts. */
    public static final double kElevatorKG = 0.0;
    public static final double kElevatorEncoderPositionConversionFactor = 1;
    public static final double kElevatorEncoderVelocityConversionFactor = kElevatorEncoderPositionConversionFactor
        / 60.0;
  }

  public static final class AnglerConfig {
    public static final double kAnglerP = 0.0;
    public static final double kAnglerI = 0.0;
    public static final double kAnglerD = 0.0;
    /** Gravity term in volts. */
    public static final double kAnglerKG = 0.0;
    public static final double kAnglerEncoderPositionConversionFactor = 1;
    public static final double kAnglerEncoderVelocityConversionFactor = kAnglerEncoderPositionConversionFactor / 60.0;
  }

  public static final class FlickerConfig {
    public static final double kFlickerP = 0.0;
    public static final double kFlickerI = 0.0;
    public static final double kFlickerD = 0.0;
    /** Gravity term in volts. */
    public static final double kFlickerKG = 0.0;
    public static final double kFlickerEncoderPositionConversionFactor = 1;
    public static final double kFlickerEncoderVelocityConversionFactor = kFlickerEncoderPositionConversionFactor / 60.0;
  }

  public static final class AlgaeWristConfig {
    public static final double kAlgaeP = 0.0;
    public static final double kAlgaeI = 0.0;
    public static final double kAlgaeD = 0.0;
    /** Gravity term in volts. */
    public static final double kAlageKG = 0.0;
  }
}
