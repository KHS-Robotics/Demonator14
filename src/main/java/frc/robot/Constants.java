package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
  public static final double DRIVE_JOYSTICK_SENSITIVITY = 0.5;

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
  public static final double SDS_L2_DRIVE_GEARING = 6.75;
  public static final double DRIVE_POS_ENCODER = (2 * Math.PI * WHEEL_RADIUS_METERS) / SDS_L2_DRIVE_GEARING;
  public static final double DRIVE_VEL_ENCODER = DRIVE_POS_ENCODER / 60.0;

  public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(Units.inchesToMeters(10.7),
      Units.inchesToMeters(11.7));
  public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(Units.inchesToMeters(10.7),
      Units.inchesToMeters(-11.7));
  public static final Translation2d REAR_LEFT_OFFSET = new Translation2d(Units.inchesToMeters(-10.7),
      Units.inchesToMeters(11.7));
  public static final Translation2d REAR_RIGHT_OFFSET = new Translation2d(Units.inchesToMeters(-10.7),
      Units.inchesToMeters(-11.7));

  public static final double FRONT_LEFT_PIVOT_OFFSET_DEGREES = 225;
  public static final double FRONT_RIGHT_PIVOT_OFFSET_DEGREES = 135;
  public static final double REAR_LEFT_PIVOT_OFFSET_DEGREES = 315;
  public static final double REAR_RIGHT_PIVOT_OFFSET_DEGREES = 45;

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
