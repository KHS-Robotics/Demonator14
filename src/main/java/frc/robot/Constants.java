package frc.robot;

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
   * Joystick (HID) configurations.
   */
  public static final class HIDConfig {
    /** Ranges from [0, 1] where 0 is full linear and 1 is full cubic. */
    public static final double kJoystickSensitivity = 0.5;

    /** Deadband since joysticks vary in how well they snap back to zero. */
    public static final double kJoystickDeadband = 0.035;
  }

  public static final class AlgaeWristConfig {
    public static final double kAlgaeP = 0.0;
    public static final double kAlgaeI = 0.0;
    public static final double kAlgaeD = 0.0;
    /** Gravity term in volts. */
    public static final double kAlageKG = 0.0;
  }
}
