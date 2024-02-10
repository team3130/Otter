// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerVibration {
  XboxController driverController = new XboxController(0); // Driver Controller
  XboxController operatorController = new XboxController(1); //Operator Controller
  double vibrationIntensity = 0.5; // Example intensity, can be set between 0 and 1
  double delayInSeconds = 5.0; //time in seconds before vibration
  double vibrationDuration = 2.0; //how long the controller will vibrate for
  public Timer DriverVibrationTimer;
  public void VibrateDriver() {
    // Activate vibration
    driverController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    driverController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);
  }
  public void StopVibrateDriver(){
    // Stop vibration
    driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }
  public void VibrateOperator(){
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);
  }
  public void StopVibrateOperator(){
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }
  public void TimedVibrateDriver(){
    driverController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    driverController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

    Timer.delay(vibrationDuration);

    driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }

  public void TimedVibrateOperator(){
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

    Timer.delay(vibrationDuration);

    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }
}