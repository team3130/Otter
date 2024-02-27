// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Intake.SmartSpintake;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED ampLEDs;
  AddressableLED shooterLEDs;
  AddressableLED leftClimberLEDs;
  AddressableLED rightClimberLEDs;
  AddressableLEDBuffer ampBuffer;
  AddressableLEDBuffer shooterBuffer;
  AddressableLEDBuffer leftClimberBuffer;
  AddressableLEDBuffer rightClimberBuffer;

  private final int ampLength = 20;
  private final int shooterLength = 23;
  private final int climberLength = 5;
  private int testShuff = 0;

  public LEDSubsystem() {
    ampLEDs = new AddressableLED(0); // PWM Port
    shooterLEDs = new AddressableLED(1); // PWM Port
    leftClimberLEDs = new AddressableLED(2); // PWM Port
    rightClimberLEDs = new AddressableLED(3); // PWM Port

    ampBuffer = new AddressableLEDBuffer(ampLength); // set length once
    ampLEDs.setLength(ampBuffer.getLength());



    // Set the data
    ampLEDs.setData(ampBuffer);
    ampLEDs.start();
  }

  /* HSV: h == set the value, s == saturation, v == brightness value
    Red: 0 - 60
    Yellow: 61 - 120
    Green: 121 - 180
    Cyan: 181 - 240
    Blue: 241 - 300
    Magenta: 301 and 360 degrees.
  */

  public void testShuffleboard() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, testShuff, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void defaultYellow() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 80, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void red() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 0, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void greenRobot() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 150, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void greenShooter() {
    for (var i = 5; i < 15; i++) {
      ampBuffer.setHSV(i, 150, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void blue() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 260, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void selectGreen() {
    for (var i = 20; i < 40; i++) {
      ampBuffer.setHSV(i, 150, 255, 128);
    }
    led.setData(ampBuffer);
  }

  public void reset() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 0, 255, 0);
    }
    led.setData(ampBuffer);
  }

  public void rainbow() {
    int rainbowFirstPixelHue = 20;
    // For every pixel
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ampBuffer.getLength())) % 180;
      ampBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    led.setData(ampBuffer);
  }

  public int getTestShuff() { return testShuff; }
  public void setTestShuff(long lol) { testShuff = (int) lol; }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LED");
    builder.addIntegerProperty("Test Color", this::getTestShuff, this::setTestShuff);
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
