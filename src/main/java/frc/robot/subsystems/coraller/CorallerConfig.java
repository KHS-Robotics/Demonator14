package frc.robot.subsystems.coraller;

final class CorallerConfig {
  protected final class ElevatorConfig {
    public static final double kElevatorP = 0.5;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.07;
    /** Gravity term in volts. */
    public static final double kElevatorKG = 0.4;
    public static final double kElevatorSprocketRadiusInches = 0.88;
    public static final double kElevatorGearing = 15.0;
    public static final double kElevatorEncoderPositionConversionFactor = (4*(Math.PI)*kElevatorSprocketRadiusInches)/kElevatorGearing;
    public static final double kElevatorEncoderVelocityConversionFactor = kElevatorEncoderPositionConversionFactor
        / 60.0;
  }

  protected final class AnglerConfig {
    public static final double kAnglerP = 0.12;
    public static final double kAnglerI = 0.0;
    public static final double kAnglerD = 0.0;
    /** Gravity term in volts. */
    public static final double kAnglerKG = 0.25;
    /** Additional gravity term in volts for coral. */
    public static final double kAnglerCoralKG = 0.065;
    public static final double kAnglerOffset = -180.0;
  }
}
