// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

public class XboxControllerVibration {
  XboxController driverController = new XboxController(0); // Driver Controller
  XboxController operatorController = new XboxController(1); //Operator Controller
  double vibrationIntensity = 0.8; // intensity of the larger vibrations. Can be set between 1 and 0
  double smallVibrationIntensity = 0.4; // intensity of the smaller vibrations. Can be set between 1 and 0
  double delayInSeconds = 5.0; //time in seconds before vibration
  double vibrationDuration = 10.0; //how long the controller will vibrate for
  boolean smallTimedVibrateDriverBool = true;
  boolean timedVibrateDriverBool = true;
  boolean smallTimedVibrateOperatorBool = true;
  boolean timedVibrateOperatorBool = true;

  public boolean getSmallTimedVibrateDriverBool(){
    return smallTimedVibrateDriverBool;
  }
  public boolean getTimedVibrateDriverBool(){
    return timedVibrateDriverBool;
  }
  public boolean getSmallTimedVibrateOperatorBool(){
    return smallTimedVibrateOperatorBool;
  }
  public boolean getTimedVibrateOperatorBool(){
    return timedVibrateOperatorBool;
  }
  public double getVibrationIntensity() {
    return vibrationIntensity;
  }
  public double getSmallVibrationIntensity(){
    return smallVibrationIntensity;
  }
  public double getVibrationDuration(){
    return vibrationDuration;
  }
  public double getDelayInSeconds(){
    return delayInSeconds;
  }
  public void setSmallTimedVibrateDriverBool(boolean SmallTimedDriver){
    smallTimedVibrateDriverBool = SmallTimedDriver;
  }
  public void setTimedVibrateDriverBool(boolean TimedDriver){
    timedVibrateDriverBool = TimedDriver;
  }
  public void setSmallTimedVibrateOperatorBool(boolean SmallTimedOperator){
    smallTimedVibrateOperatorBool = SmallTimedOperator;
  }
  public void setTimedVibrateOperatorBool(boolean TimedOperator){
    timedVibrateOperatorBool = TimedOperator;
  }
  public void setVibrationIntensity(double intensity){
    vibrationIntensity = intensity;
  }
  public void setSmallVibrationIntensity(double SmallIntensity){
    smallVibrationIntensity = SmallIntensity;
  }
  public void setVibrationDuration(double duration){
    vibrationDuration = duration;
  }
  public void setDelayInSeconds(double delay){
    delayInSeconds = delay;
  }

  public void VibrateDriver() {
    // Activate vibration
    RobotContainer.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, vibrationIntensity);
    RobotContainer.driverController.setRumble(GenericHID.RumbleType.kRightRumble, vibrationIntensity);
  }
  public void StopVibrateDriver(){
    // Stop vibration
    RobotContainer.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    RobotContainer.driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }
  public void VibrateOperator(){
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);
  }
  public void StopVibrateOperator(){
    operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }

  public void SmallTimedVibrateDriver(){
    if (smallTimedVibrateDriverBool) {
      driverController.setRumble(XboxController.RumbleType.kLeftRumble, smallVibrationIntensity);
      driverController.setRumble(XboxController.RumbleType.kRightRumble, smallVibrationIntensity);

      //Timer.delay(vibrationDuration);

      //driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
      //driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
    }
  }
  public void TimedVibrateDriver(){
    if (timedVibrateDriverBool) {
      //Timer.delay(delayInSeconds);
      driverController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
      driverController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

      //Timer.delay(vibrationDuration);

      //driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
      //driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
    }
  }

  public void SmallTimedVibrateOperator(){
    if (smallTimedVibrateOperatorBool) {
      operatorController.setRumble(XboxController.RumbleType.kLeftRumble, smallVibrationIntensity);
      operatorController.setRumble(XboxController.RumbleType.kRightRumble, smallVibrationIntensity);

      //Timer.delay(vibrationDuration);

      //operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
      //operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
    }
  }
  public void TimedVibrateOperator(){
    if (timedVibrateOperatorBool) {
      operatorController.setRumble(XboxController.RumbleType.kLeftRumble, vibrationIntensity);
      operatorController.setRumble(XboxController.RumbleType.kRightRumble, vibrationIntensity);

      //Timer.delay(vibrationDuration);

      //operatorController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
      //operatorController.setRumble(XboxController.RumbleType.kRightRumble, 0);
    }
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vibration");

    builder.addDoubleProperty("Large vibration intensity", this::getVibrationIntensity, this::setVibrationIntensity);
    builder.addDoubleProperty("Small vibration intensity", this::getSmallVibrationIntensity, this::setSmallVibrationIntensity);
    builder.addDoubleProperty("vibration duration", this::getVibrationDuration, this::setVibrationDuration);
    builder.addDoubleProperty("vibration delay", this::getDelayInSeconds, this::setDelayInSeconds);
  }
}