package frc.robot.subsystems.led;

import java.awt.Color;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

/**
 * Originally from 2024: https://github.com/KHS-Robotics/Demonator13/blob/main/src/main/java/frc/robot/subsystems/LEDStrip.java
 */
public class LEDStrip {
  Thread t;
  AddressableLED strip;
  AddressableLEDBuffer buffer;
  int numberSections;
  int counter;
  int ticksPerSecond = 20;

  float speedFactor = 1f;
  float sections = 5f;
  float currentPosition = 0f;
  Color[] pixelArray;

  private final BooleanSupplier isAbleToAlignLeft, isAbleToAlignRight, isAbleToAlignCoralStation;

  public LEDStrip(BooleanSupplier isAbleToAlignLeft, BooleanSupplier isAbleToAlignRight, BooleanSupplier isAbleToAlignCoralStation) {
    this.isAbleToAlignLeft = isAbleToAlignLeft;
    this.isAbleToAlignRight = isAbleToAlignRight;
    this.isAbleToAlignCoralStation = isAbleToAlignCoralStation;

    pixelArray = new Color[LEDConfig.LED_LENGTH];
    for (int i = 0; i < pixelArray.length; i++) {
      pixelArray[i] = new Color(0);
    }

    // multithreading this has got to be genius
    t = new Thread(() -> {
      long lastTime = System.nanoTime();
      double delta = 0;
      // very accurate loop, is this bad for performance?
      while (!Thread.interrupted()) {
        double ns = 1000000000 / (double) ticksPerSecond;
        long now = System.nanoTime();
        delta += (now - lastTime) / ns;
        lastTime = now;
        if (delta >= 1) {
          update();
          delta--;
        }
      }
    });
    
    strip = new AddressableLED(RobotMap.LED_PORT);
    strip.setLength(LEDConfig.LED_LENGTH);

    buffer = new AddressableLEDBuffer(LEDConfig.LED_LENGTH);
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB(i, 255, 255, 255);
    }
    strip.setData(buffer);
    strip.start();

    this.numberSections = LEDConfig.LED_LENGTH;
    t.start();
  }

  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }

  public void fillBuffer(Color[] colors) {
    for (int i = 0; i < colors.length; i++) {
      setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }
  }

  public void setPixelColorHSB(int i, float h, float s, float b) {
    Color c = Color.getHSBColor(h, s, b);
    setRGB(i, c.getRed(), c.getGreen(), c.getBlue());
  }

  public float[] getHSB(Color c) {
    float[] rgb = new float[3];
    rgb = c.getRGBColorComponents(rgb);

    rgb[0] /= 255f;
    rgb[1] /= 255f;
    rgb[2] /= 255f;

    float cmax = Math.max(rgb[0], Math.max(rgb[1], rgb[2]));
    float cmin = Math.min(rgb[0], Math.min(rgb[1], rgb[2]));
    float diff = cmax - cmin;
    float h = -1, s;

    if (cmax == cmin) {
      h = 0;
    }

    else if (cmax == rgb[0]) {
      h = (60 * ((rgb[1] - rgb[2]) / diff) + 360) % 360;
    }

    // if cmax equal g then compute h
    else if (cmax == rgb[1]) {
      h = (60 * ((rgb[2] - rgb[0]) / diff) + 120) % 360;
    }

    // if cmax equal b then compute h
    else if (cmax == rgb[2]) {
      h = (60 * ((rgb[0] - rgb[1]) / diff) + 240) % 360;
    }

    // if cmax equal zero
    if (cmax == 0) {
      s = 0;
    } else {
      s = (diff / cmax) * 100;
    }

    h /= 360;
    s /= 100;
    float b = cmax * 255;

    return new float[] { h, s, b };
  }

  public void setAllRed() {
    ticksPerSecond = 5;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB(i, 255, 0, 0);
    }
  }

  public void setAllBlue() {
    ticksPerSecond = 5;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB(i, 0, 0, 255);
    }
  }

  public void setAllOff() {
    ticksPerSecond = 5;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB(i, 0, 0, 0);
    }
  }

  public void runBlue() {
    ticksPerSecond = 20;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB((i + counter) % LEDConfig.LED_LENGTH, 0, 0,
          (int) ((-Math.cos((2 * Math.PI * 2 * i) / LEDConfig.LED_LENGTH)) + 1) * 50);
    }
  }

  public void runRed() {
    ticksPerSecond = 20;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setRGB((i + counter) % LEDConfig.LED_LENGTH,
          (int) ((-Math.cos((2 * Math.PI * 2 * i) / LEDConfig.LED_LENGTH)) + 1) * 50, 0, 0);
    }
  }

  public void runAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      runRainbow();
    } else {
      if (alliance.get() == Alliance.Blue) {
        runBlue();
      } else {
        runRed();
      }
    }
  }

  public void runRainbow() {
    ticksPerSecond = 20;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setHSV((i + counter) % LEDConfig.LED_LENGTH, (int) (((double) i / LEDConfig.LED_LENGTH) * 180), 255,
          255);

    }
  }

  public void runSilly() {
    ticksPerSecond = 5;
    for (int i = 0; i < LEDConfig.LED_LENGTH; i++) {
      setHSV(i, (int) (Math.random() * 180), 255, 255);
    }
  }

  // run square wave of alliance color
  public void runDisabled() {
    ticksPerSecond = 20;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        runSquareWave(Color.BLUE, -0.4f, 8f);
      } else if (alliance.get() == Alliance.Red) {
        runSquareWave(Color.RED, -0.4f, 8f);
      } else {
        runSquareWave(Color.WHITE, -0.4f, 8f);
      }
    } else {
      runRainbow();
    }
  }

  // BLUE
  public void runCanAlignLeftReef() {
    ticksPerSecond = 20;
    if (isAbleToAlignLeft.getAsBoolean()) {
      // let there be blue
    } else {
      // turn off blue
    }
  }

  // RED
  public void runCanAlignRightReef() {
    ticksPerSecond = 20;
    if (isAbleToAlignRight.getAsBoolean()) {
      // let there be red
    } else {
      // turn off red
    }
  }

  // YELLOW
  public void runCanAlignCoralStation() {
    ticksPerSecond = 20;
    if (isAbleToAlignCoralStation.getAsBoolean()) {
      // let there be yellow
    } else {
      // turn off yellow
    }
  }

  public void runSquareWave(Color c, float speed, float sections) {
    float[] hsb = getHSB(c);
    this.speedFactor = speed;
    this.sections = sections;
    currentPosition += speedFactor;

    for (int i = 0; i < pixelArray.length; i++) {
      float adjustedPosition = (i + currentPosition) % pixelArray.length;

      if (adjustedPosition < 0) {
        adjustedPosition += pixelArray.length;
      }

      float j = (float) Math.abs(Math.pow(Math.sin((((Math.PI / 2) * adjustedPosition) * (0.02 * sections))), 2));

      if (j > 0.5) {
        j = hsb[2];
      } else {
        j = 0;
      }

      setPixelColorHSB(i, hsb[0], hsb[1], j);
    }
  }

  public void update() {
    if (RobotState.isDisabled()) {
      runAllianceColor();
    }
    else if(RobotState.isEnabled()) {
      runCanAlignLeftReef();
      runCanAlignRightReef();
      runCanAlignCoralStation();
    }
    else {
      runRainbow();
    }

    strip.setData(buffer);
    counter++;
  }
}