package frc.robot.subsystems.coraller;

public class CorallerConfig {

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


}
