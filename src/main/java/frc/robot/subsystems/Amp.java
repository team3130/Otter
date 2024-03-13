// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Amp extends SubsystemBase {
  private final DigitalInput ampLimit;
  private final DigitalInput ampBeam;
  private final WPI_TalonSRX ampLiftingMotor;
  private final WPI_TalonSRX ampSpinningMotor;
  private double intakeAmpSpeed = 1;
  private double ampLiftSpeed = 0.5;
  private double ampLowerSpeed = -0.5;
  private double outtakeAmpSpeed = -1;
  private int encoderMaxTicks = 215;
  private int highSetpoint = 200;
  private int midSetpoint = 100;
  private int lowSetpoint = 15;
  private ProfiledPIDController ampController;
  private double P = 0;
  private double I = 0;
  private double D = 0;
  private TrapezoidProfile.Constraints constraints;
  private double maxVelo= 1;
  private double maxAcc = 1;
  private boolean hasZeroed = false;

  public Amp() {
    ampLimit = new DigitalInput(Constants.IDs.ampLimitDIO);
    ampBeam = new DigitalInput(Constants.IDs.ampBeamDIO);

    ampLiftingMotor = new WPI_TalonSRX(Constants.CAN.ampLiftMotor);
    ampSpinningMotor = new WPI_TalonSRX(Constants.CAN.ampSpinMotor);

    ampLiftingMotor.configFactoryDefault();
    ampSpinningMotor.configFactoryDefault();

    ampSpinningMotor.setNeutralMode(NeutralMode.Brake);
    ampLiftingMotor.setNeutralMode(NeutralMode.Brake);

    ampSpinningMotor.enableVoltageCompensation(true);
    ampSpinningMotor.configVoltageCompSaturation(3);

    ampLiftingMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    ampLiftingMotor.setInverted(false);
    ampSpinningMotor.setInverted(false);

    ampSpinningMotor.enableCurrentLimit(true);
    ampSpinningMotor.configContinuousCurrentLimit(20);
    ampSpinningMotor.configPeakCurrentLimit(0);

    ampLiftingMotor.enableCurrentLimit(true);
    ampLiftingMotor.configContinuousCurrentLimit(20);
    ampLiftingMotor.configPeakCurrentLimit(0);


    constraints = new TrapezoidProfile.Constraints(maxVelo, maxAcc);
    ampController = new ProfiledPIDController(P, I, D, constraints);
  }
  public boolean getAmpBeamHasNote(){
    return ampBeam.get();
  }
  public boolean getHasZeroed(){
    return hasZeroed;
  }
  public void setHasZeroedTrue(){
    hasZeroed = true;
  }

  public void resetControllerHigh(){
    ampController.reset(getEncoderPosition());
    ampController.setGoal(highSetpoint);
    ampController.setTolerance(15);
    ampController.setPID(P, I, D);
  }
  public void resetControllerMid(){
    ampController.reset(getEncoderPosition());
    ampController.setGoal(midSetpoint);
    ampController.setTolerance(15);
    ampController.setPID(P, I, D);
  }

  public void resetControllerLow(){
    ampController.reset(getEncoderPosition());
    ampController.setGoal(lowSetpoint);
    ampController.setTolerance(15);
    ampController.setPID(P, I, D);
  }

  public int getHighSetpoint(){
    return highSetpoint;
  }
  public int getMidSetpoint() { return midSetpoint; }
  public int getLowSetpoint(){
    return lowSetpoint;
  }


  public void setHighSetpoint(long set){
    highSetpoint = (int) set;
  }
  public void setMidSetpoint(long set){
    midSetpoint = (int) set;
  }
  public void setLowSetpoint(long set){
    lowSetpoint = (int) set;
  }



  public void runController(){
    ampController.calculate(getEncoderPosition());
  }
  public boolean isAtSetpoint(){
    return ampController.atSetpoint();
  }

  public void intakeAmp() {
    ampSpinningMotor.set(ControlMode.PercentOutput, intakeAmpSpeed);
  }
  public void outtakeAmp() {
    ampSpinningMotor.set(ControlMode.PercentOutput, outtakeAmpSpeed);
  }
  public void ampSpinningMotorStop() {
    ampSpinningMotor.set(ControlMode.PercentOutput, 0);
  }
  public void ampLiftingMotorStop() {
    ampLiftingMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetEncoder(){
    ampLiftingMotor.setSelectedSensorPosition(0);
  }

  public boolean getLimitSwitch() {
    return !ampLimit.get();
  }
  public double getEncoderPosition() { return ampLiftingMotor.getSelectedSensorPosition();}

  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }
  public int getEncoderMax(){
    return encoderMaxTicks;
  }
  public void setEncoderMax(long max){
    encoderMaxTicks = (int) max;
  }
  public double getAmpLiftSpeed(){
    return ampLiftSpeed;
  }
  public double getAmpLowerSpeed(){
    return ampLowerSpeed;
  }
  public void setAmpLiftSpeed(double speed){
    ampLiftSpeed = speed;
  }
  public void setAmpLowerSpeed(double speed){
    ampLowerSpeed = speed;
  }
  public double getOuttakeAmpSpeed() { return outtakeAmpSpeed; }
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }
  public void setOuttakeAmpSpeed(double speed) {
    outtakeAmpSpeed = speed;
  }

  public void manualAmpLiftUp(){
    ampLiftingMotor.set(ampLiftSpeed);
  }
  public void manualAmpLowerDown(){
    ampLiftingMotor.set(ampLowerSpeed);
  }



  @Override
  public void periodic() {
  }

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Amp");
      builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
      builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
      builder.addDoubleProperty("Lift Amp Speed", this::getAmpLiftSpeed, this::setAmpLiftSpeed);
      builder.addDoubleProperty("Lower Amp Speed", this::getAmpLowerSpeed, this::setAmpLowerSpeed);
      builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
      builder.addIntegerProperty("Encoder Max", this::getEncoderMax, this::setEncoderMax);
      builder.addDoubleProperty("Encoder Position", this::getEncoderPosition, null);
      builder.addIntegerProperty("high setpoint", this::getHighSetpoint, this::setHighSetpoint);
      builder.addIntegerProperty("low setpoint", this::getLowSetpoint, this::setLowSetpoint);
      builder.addIntegerProperty("mid setpoint", this::getMidSetpoint, this::setMidSetpoint);
      builder.addBooleanProperty("Is At Setpoint", this::isAtSetpoint, null);
      builder.addBooleanProperty("has zeroed", this::getHasZeroed, null);
      builder.addBooleanProperty("amp beam has note", this::getAmpBeamHasNote, null);
    }
  }
}