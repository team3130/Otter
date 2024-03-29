// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  AddressableLED shooterLEDs;
  private final int LEDLength = 23;
  private int rainbowFirstPixelHue = 20;
  private long shuffleboardTest;
  private final int pwmPort = 0;

  AddressableLEDBuffer shooterBuffer;
  //private ComplexWidget whiteLEDs;

  public LEDs() {
    shooterLEDs = new AddressableLED(pwmPort); // PWM Port
    shooterBuffer = new AddressableLEDBuffer(LEDLength);
    shooterLEDs.setLength(shooterBuffer.getLength());
    shooterLEDs.setData(shooterBuffer);
    shooterLEDs.start();
    // whiteLEDs = Shuffleboard.getTab("LEDs").add("White LEDs", new WhiteLEDs(this));
  }

  public void yellowRobot() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, Constants.LEDColors.yellowHSV, 255, 200);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void purpleRobot() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, Constants.LEDColors.purpleHSV, 255, 200);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void redRobot() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, Constants.LEDColors.redHSV, 255, 200);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void movingRainbow() {
    // For every pixel
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / shooterBuffer.getLength())) % 180;
      shooterBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    shooterLEDs.setData(shooterBuffer);
  }


  public void white() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setLED(i, Color.kFloralWhite);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void staticRainbow() {
    // For every pixel
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / shooterBuffer.getLength())) % 180;
      shooterBuffer.setHSV(i, hue, 255, 200);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    shooterLEDs.setData(shooterBuffer);
  }

  public void shuffleColor() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, getShuffleboardTest(), 255, 200);
    }
    shooterLEDs.setData(shooterBuffer);
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
