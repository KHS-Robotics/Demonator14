package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

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
  /** Ranges from [0, 1] where 0 is full linear and 1 is full cubic. */
  public static final double kJoystickSensitivity = 0.5;

  /**
   * Configurations for PhotonVision.
   */
  public static class PhotonVisionConfig {
    public static final String kCameraName = "4342_AprilTag_1";

    /**
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-a-photonposeestimator
     */
    public static final Transform3d kRobotToCamera = new Transform3d(Units.inchesToMeters(7.6882),
        Units.inchesToMeters(13.0), Units.inchesToMeters(12.1545), new Rotation3d(0, Math.toRadians(-60), 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2024Crescendo);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  /**
   * Configurations for the swerve drive.
   */
  public static class SwerveDriveConfig {
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(2);
    public static final double SDS_L2_DRIVE_GEARING = 6.75;
    public static final double kDriveEncoderPositionConversionFactor = (2 * Math.PI * kDriveWheelRadiusMeters)
        / SDS_L2_DRIVE_GEARING;
    public static final double kDriveEncoderVelocityConversionFactor = kDriveEncoderPositionConversionFactor / 60.0;

    public static final Translation2d kFrontLeftModuleOffset = new Translation2d(Units.inchesToMeters(10.7),
        Units.inchesToMeters(11.7));
    public static final Translation2d kFrontRightModuleOffset = new Translation2d(Units.inchesToMeters(10.7),
        Units.inchesToMeters(-11.7));
    public static final Translation2d kRearLeftModuleOffset = new Translation2d(Units.inchesToMeters(-10.7),
        Units.inchesToMeters(11.7));
    public static final Translation2d kRearRightModuleOffset = new Translation2d(Units.inchesToMeters(-10.7),
        Units.inchesToMeters(-11.7));

    // individual offsets after calibrating each module
    public static final double kFrontLeftPivotOffsetDegrees = 225;
    public static final double kFrontRightPivotOffsetDegrees = 135;
    public static final double kRearLeftPivotOffsetDegrees = 315;
    public static final double kRearRightPivotOffsetDegrees = 45;

    public static final double DRIVE_MODULE_P = 0.01;
    public static final double DRIVE_MODULE_I = 0;
    public static final double DRIVE_MODULE_D = 0;
    public static final double DRIVE_MODULE_KS = 0.11408;
    public static final double DRIVE_MODULE_KV = 3.2717;
    public static final double DRIVE_MODULE_KA = 0.17904;

    public static final double DRIVE_MODULE_PIVOT_P = 0.007;
    public static final double DRIVE_MODULE_PIVOT_I = 0.0;
    public static final double DRIVE_MODULE_PIVOT_D = 0.0001;

    public static final double DRIVE_ANGLE_P = 0.005;
    public static final double DRIVE_ANGLE_I = 0;
    public static final double DRIVE_ANGLE_D = 0;

    public static final double DRIVE_X_P = 0.3;
    public static final double DRIVE_X_I = 0;
    public static final double DRIVE_X_D = 0;

    public static final double DRIVE_Y_P = 0.3;
    public static final double DRIVE_Y_I = 0;
    public static final double DRIVE_Y_D = 0;

    public static final double DRIVE_PATHING_TRANSLATION_P = 4.0;
    public static final double DRIVE_PATHING_TRANSLATION_I = 0.0;
    public static final double DRIVE_PATHING_TRANSLATION_D = 0.3;

    public static final double DRIVE_PATHING_ROTATION_P = 1.5;
    public static final double DRIVE_PATHING_ROTATION_I = 0.0;
    public static final double DRIVE_PATHING_ROTATION_D = 0.8;
  }
}
