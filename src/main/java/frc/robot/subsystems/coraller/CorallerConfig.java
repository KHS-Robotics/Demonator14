package frc.robot.subsystems.coraller;

final class CorallerConfig {
  protected final class ElevatorConfig {
    public static final double kElevatorP = 0.85;
    public static final double kElevatorI = 0.15;
    public static final double kElevatorD = 0.07;
    /** Gravity term in volts. */
    public static final double kElevatorKG = 0.4;
    public static final double kElevatorSprocketRadiusInches = 0.88;
    public static final double kElevatorGearing = 15.0;
    public static final double kElevatorEncoderPositionConversionFactor = (4*(Math.PI)*kElevatorSprocketRadiusInches)/kElevatorGearing;
    public static final double kElevatorEncoderVelocityConversionFactor = kElevatorEncoderPositionConversionFactor
        / 60.0;

    public static final double kElevatorAbsoluteEncoderMinVoltage = 0.3918;
    public static final double kElevatorAbsoluteEncoderMaxVoltage = 1.6100;
  }

  protected final class AnglerConfig {
    public static final double kAnglerP = 0.06;
    public static final double kAnglerI = 0.01;
    public static final double kAnglerD = 0.0015;
    /** Gravity term in volts. */
    public static final double kAnglerKG = 0.25;
    /** Additional gravity term in volts for coral. */
    public static final double kAnglerCoralKG = 0.065;
    public static final double kAnglerOffset = -137;
  }
}
