// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerVibration {
  public void Vibration () {
    // Initialize Xbox controller with port number
    XboxController xboxController = new XboxController(0); // Change port number if necessary

    // Delay before activating vibration (in seconds)
    double delayInSeconds = 5.0;

    // Wait for the specified delay
    Timer.delay(delayInSeconds);

    // Set vibration intensity (ranges from 0 to 1)
    double vibrationIntensity = 0.5; // Example intensity, can be adjusted

    // Activate vibration
    xboxController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    xboxController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

    // Wait for a duration of vibration (in seconds)
    double vibrationDuration = 2.0; // Example duration, can be adjusted
    Timer.delay(vibrationDuration);

    // Stop vibration
    xboxController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    xboxController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }
}