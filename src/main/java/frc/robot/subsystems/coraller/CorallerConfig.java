package frc.robot.subsystems.coraller;

final class CorallerConfig {
  protected final class ElevatorConfig {
    public static final double kElevatorP = 0.0;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    /** Gravity term in volts. */
    public static final double kElevatorKG = 1.5;
    public static final double kElevatorKGSlowDescent = 0.8;
    public static final double kElevatorKGAscent = 1.9;
    public static final double kElevatorSprocketRadiusInches = 0.88;
    public static final double kElevatorGearing = 3.0;
    public static final double kElevatorEncoderPositionConversionFactor = (4*(Math.PI)*kElevatorSprocketRadiusInches)/kElevatorGearing;
    public static final double kElevatorEncoderVelocityConversionFactor = kElevatorEncoderPositionConversionFactor
        / 60.0;
  }

  protected final class AnglerConfig {
    public static final double kAnglerP = 0.0;
    public static final double kAnglerI = 0.0;
    public static final double kAnglerD = 0.0;
    /** Gravity term in volts. */
    public static final double kAnglerKG = 0.25;
    /** Additional gravity term in volts for coral. */
    public static final double kAnglerCoralKG = 0.065;
    public static final double kAnglerOffset = 180.0;
  }

  protected final class FlickerConfig {
    public static final double kFlickerP = 0.0;
    public static final double kFlickerI = 0.0;
    public static final double kFlickerD = 0.0;
    /** Gravity term in volts. */
    public static final double kFlickerKG = 0.0;
    public static final double kFlickerEncoderPositionConversionFactor = 1;
    public static final double kFlickerEncoderVelocityConversionFactor = kFlickerEncoderPositionConversionFactor / 60.0;
  }
}
