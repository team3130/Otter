// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpLEDs extends SubsystemBase {
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

  public AmpLEDs() {
    /*
    ampLEDs = new AddressableLED(0); // PWM Port
    ampBuffer = new AddressableLEDBuffer(ampLength); // set length once
    ampLEDs.setLength(ampBuffer.getLength());
    ampLEDs.setData(ampBuffer); // set data
    ampLEDs.start();

    //leftClimberLEDs = new AddressableLED(1); // PWM Port
    /*
    rightClimberLEDs = new AddressableLED(2); // PWM Port

     */


    /*

    leftClimberBuffer = new AddressableLEDBuffer(climberLength);
    leftClimberLEDs.setLength(leftClimberBuffer.getLength());

    rightClimberBuffer = new AddressableLEDBuffer(climberLength);
    rightClimberLEDs.setLength(rightClimberBuffer.getLength());

     */

    // Set the data

    /*

    leftClimberLEDs.setData(leftClimberBuffer);
    leftClimberLEDs.start();

    rightClimberLEDs.setData(rightClimberBuffer);
    rightClimberLEDs.start();

     */
  }

  /* HSV: h == set the value, s == saturation, v == brightness value
    100 - blue
    15 - yellow
    25 - green
  */

  public void testShuffleboard() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, testShuff, 255, 128);
    }
    ampLEDs.setData(ampBuffer);
  }

  public void defaultYellow() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 10, 255, 255);
    }

    /*
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 10, 255, 128);
    }
    for (var i = 0; i < rightClimberBuffer.getLength(); i++) {
      rightClimberBuffer.setHSV(i, 80, 255, 128);
    }

     */

    ampLEDs.setData(ampBuffer);
    /*

    shooterLEDs.setData(shooterBuffer);
    rightClimberLEDs.setData(rightClimberBuffer);

     */
  }

  /*
  public void red() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 0, 255, 128);
    }
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 0, 255, 128);
    }
    for (var i = 0; i < leftClimberBuffer.getLength(); i++) {
      leftClimberBuffer.setHSV(i, 0, 255, 128);
    }
    for (var i = 0; i < rightClimberBuffer.getLength(); i++) {
      rightClimberBuffer.setHSV(i, 0, 255, 128);
    }

    ampLEDs.setData(ampBuffer);
    shooterLEDs.setData(shooterBuffer);
    leftClimberLEDs.setData(leftClimberBuffer);
    rightClimberLEDs.setData(rightClimberBuffer);
  }

  public void greenRobot() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 150, 255, 128);
    }
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 150, 255, 128);
    }
    for (var i = 0; i < leftClimberBuffer.getLength(); i++) {
      leftClimberBuffer.setHSV(i, 150, 255, 128);
    }
    for (var i = 0; i < rightClimberBuffer.getLength(); i++) {
      rightClimberBuffer.setHSV(i, 150, 255, 128);
    }

    ampLEDs.setData(ampBuffer);
    shooterLEDs.setData(shooterBuffer);
    leftClimberLEDs.setData(leftClimberBuffer);
    rightClimberLEDs.setData(rightClimberBuffer);
  }

  public void greenShooter() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 150, 255, 128);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void reset() {
    for (var i = 0; i < ampBuffer.getLength(); i++) {
      ampBuffer.setHSV(i, 0, 255, 0);
    }
    ampLEDs.setData(ampBuffer);
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
    ampLEDs.setData(ampBuffer);
  }

   */

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
