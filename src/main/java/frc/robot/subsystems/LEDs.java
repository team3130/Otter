// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  AddressableLED LEDs;
  private final int LEDLength = 37;
  private int rainbowFirstPixelHue = 20;
  private long shuffleboardTest;
  private final int pwmPort = 0;

  AddressableLEDBuffer buffer;
  //private ComplexWidget whiteLEDs;

  public LEDs() {
    LEDs = new AddressableLED(pwmPort); // PWM Port
    buffer = new AddressableLEDBuffer(LEDLength);
    LEDs.setLength(buffer.getLength());
    LEDs.setData(buffer);
    LEDs.start();
    // whiteLEDs = Shuffleboard.getTab("LEDs").add("White LEDs", new WhiteLEDs(this));
  }

  public void setBackClimbers(int color) {
    /*
    for (var i = 0; i < 5; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    
     */
    for (var i = 32; i < 37; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void setBottomFrontClimbers(int color) {
    for (var i = 2; i < 5; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    for (var i = 27; i < 30; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void setTopFrontClimbers(int color) {
    if (color == -20) {
      for (var i = 0; i < 2; i++) {
        buffer.setLED(i, Color.kFloralWhite);
      }
      for (var i = 30; i < 32; i++) {
        buffer.setLED(i, Color.kFloralWhite);
      }
    } else {
      for (var i = 0; i < 2; i++) {
        buffer.setHSV(i, color, 255, 200);
      }
      for (var i = 30; i < 32; i++) {
        buffer.setHSV(i, color, 255, 200);
      }
    }
    LEDs.setData(buffer);
  }

  public void setSidebars(int color) {
    for (var i = 5; i < 13; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    for (var i = 19; i < 27; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void setBar(int color) {
    for (var i = 13; i < 19; i++) {
      buffer.setHSV(i, color, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void greenRobot() {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, Constants.LEDColors.lightGreenHSV, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void redRobot() {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, Constants.LEDColors.redHSV, 255, 200);
    }
    LEDs.setData(buffer);
  }

  public void movingRainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    LEDs.setData(buffer);
  }

  public void white() {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, Color.kFloralWhite);
    }
    LEDs.setData(buffer);
  }

  public void staticRainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 200);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    LEDs.setData(buffer);
  }

  public void shuffleColor() {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, getShuffleboardTest(), 255, 200);
    }
    LEDs.setData(buffer);
  }


  public void setShuffleboardTest(long lol) { shuffleboardTest = lol;}
  public int getShuffleboardTest() { return (int) shuffleboardTest;}

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.addIntegerProperty("shuffle", this::getShuffleboardTest, this::setShuffleboardTest);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
