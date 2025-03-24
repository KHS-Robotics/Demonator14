package frc.robot.subsystems.climber;

class ClimberConfig {
  public static final class AnchorConfig {
    public static final double kAnchorEncoderPositionConversionFactor = 360;
    public static final double kAnchorEncoderVelocityConversionFactor = kAnchorEncoderPositionConversionFactor / 60.0;
    public static final double kAnchorP = 0.0015;
    public static final double kAnchorD = 0.02;
  }
}
