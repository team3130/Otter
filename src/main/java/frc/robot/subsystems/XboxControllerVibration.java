// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerVibration {
  XboxController driverController = new XboxController(0); // Driver Controller
  XboxController operatorController = new XboxController(1); //Operator Controller
  double vibrationIntensity = 0.5; // Example intensity, can be set between 0 and 1
  double delayInSeconds = 115.0; //time in seconds before vibration
  double vibrationDuration = 2.0; //how long the controller will vibrate for

  public double getVibrationIntensity() {
    return vibrationIntensity;
  }
  public double getVibrationDuration(){
    return vibrationDuration;
  }
  public double getDelayInSeconds(){
    return delayInSeconds;
  }
  public void setVibrationIntensity(double intensity){
    vibrationIntensity = intensity;
  }
  public void setVibrationDuration(double duration){
    vibrationDuration = duration;
  }
  public void setDelayInSeconds(double delay){
    delayInSeconds = delay;
  }

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
    Timer.delay(115);
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

    Timer.delay(vibrationDuration);

    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vibration");

    builder.addDoubleProperty("Large vibration intensity", this::getVibrationIntensity, this::setVibrationIntensity);
    //add small vibration intensity export here
    builder.addDoubleProperty("vibration duration", this::getVibrationDuration, this::setVibrationDuration);
    builder.addDoubleProperty("vibration delay", this::getDelayInSeconds, this::setDelayInSeconds);
  }
}