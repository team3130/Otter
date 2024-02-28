// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterLEDs extends SubsystemBase {
  AddressableLED shooterLEDs;
  private final int shooterLength = 23;

  private int rainbowFirstPixelHue = 20;

  AddressableLEDBuffer shooterBuffer;

  // 65 == teal
  // 55 == green
  public ShooterLEDs(int pwmPort, int length) {
    shooterLEDs = new AddressableLED(pwmPort); // PWM Port
    shooterBuffer = new AddressableLEDBuffer(length);
    shooterLEDs.setLength(shooterBuffer.getLength());
    shooterLEDs.setData(shooterBuffer);
    shooterLEDs.start();
  }

  public void shooterYellow() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 10, 255, 255);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void purpleRobot() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 150, 255, 128);
    }
    shooterLEDs.setData(shooterBuffer);
  }

  public void greenRobot() {
    for (var i = 0; i < shooterBuffer.getLength(); i++) {
      shooterBuffer.setHSV(i, 65, 255, 128);
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

  public void staticRainbow() {
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

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LED");
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
