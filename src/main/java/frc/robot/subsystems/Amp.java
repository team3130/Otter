// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Amp extends SubsystemBase {
  private final DigitalInput ampLimit;
  private final WPI_TalonSRX ampMotor;
  private double intakeAmpSpeed = 1;
  private double outtakeAmpSpeed = -1;

  public Amp() {
    ampLimit = new DigitalInput(Constants.IDs.ampLimitDIO);
    ampMotor = new WPI_TalonSRX(Constants.CAN.ampMotor);

    ampMotor.configFactoryDefault();
    ampMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    ampMotor.configVoltageCompSaturation(8);
    ampMotor.setInverted(false);
  }

  public void intakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, intakeAmpSpeed);
  }
  public void outtakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, outtakeAmpSpeed);
  }
  public void ampMotorStop() {
    ampMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getLimitSwitch() {
    return !ampLimit.get();
  }
  public double getEncoderPosition() { return ampMotor.getSelectedSensorPosition();}

  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }
  public double getOuttakeAmpSpeed() { return outtakeAmpSpeed; }
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }
  public void setOuttakeAmpSpeed(double speed) {
    outtakeAmpSpeed = speed;
  }

  @Override
  public void periodic() {
  }

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Amp");
      builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
      builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
      builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
      builder.addDoubleProperty("Encoder Position", this::getEncoderPosition, null);
    }
  }
}