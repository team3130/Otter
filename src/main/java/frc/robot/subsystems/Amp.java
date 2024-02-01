// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {

  private final Solenoid pneumatic;
  private final DigitalInput ampLimit;
  private final WPI_TalonSRX ampMotor;
  private double intakeAmpSpeed = 0.1;
  private double outtakeAmpSpeed = -0.1;
  /**
  * primetime & outtaketime are the amount of seconds it takes for amp to prime (go up)
   * and to outtake (spit out note).
   */
  private double primeTime = 1;
  private double outtakeTime = 2;

  public Amp() {
    pneumatic = new Solenoid(Constants.CAN.ampPCM, PneumaticsModuleType.CTREPCM, Constants.CAN.ampChannel);
    ampLimit = new DigitalInput(Constants.CAN.ampLimitSwitch);
    ampMotor = new WPI_TalonSRX(Constants.CAN.ampMotor);
    ampMotor.configFactoryDefault();
    ampMotor.configVoltageCompSaturation(3);
    ampMotor.setInverted(false);
  }

  /**
   * @return the status of the limit switch
   */
  public boolean getLimitSwitch() {
    return ampLimit.get();
  }

  /**
   * toggles the pneumatic to prop-up the amp arm
   */
  public void primeAmp() {
    pneumatic.set(true);
  }

  /**
   * toggles the pneumatic to tuck in the amp arm
   */
  public void unPrimeAmp() {
    pneumatic.set(false);
  }

  /**
   * spins the motor to intake notes into the amp
   */
  public void intakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, intakeAmpSpeed);
  }

  /**
   * spins the motor to eject notes from the amp
   */
  public void outtakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, outtakeAmpSpeed);
  }

  /**
   * stops the motor to prevent the note from getting destroyed in the amp
   */
  public void motorStop() {
    ampMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return speed of the motor while intaking
   */
  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }

  /**
   * @return speed of the motor while outtaking
   */
  public double getOuttakeAmpSpeed() {
    return outtakeAmpSpeed;
  }

  /**
   * @return status of the pneumatic to see whether amp is propped up or not
   */
  public boolean getPneumaticState() {
    return pneumatic.get();
  }

  /**
   * @return the time required to prop up amp arm (may be useful when combining amp commands, mainly for shuffleboard)
   */
  public double getPrimeTime() {
    return primeTime;
  }

  /**
   * @return the time required to tuck in amp arm (may be useful when combining amp commands, mainly for shuffleboard)
   */
  public double getOuttakeTime() {
    return outtakeTime;
  }

  /**
   * @return sets value of amp motor's intake speed
   */
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }

  /**
   * @return sets value of amp motor's outtake speed
   */
  public void setOuttakeAmpSpeed(double speed) {
    outtakeAmpSpeed = speed;
  }

  /**
   * sets the time required to prop up amp arm (mainly for shuffleboard)
   * @param pt
   */
  public void setPrimeTime(double pt) {
    primeTime = pt;
  }

  /**
   * sets the time required to tuck in amp arm (mainly for shuffleboard)
   * @param ot
   */
  public void setOuttakeTime(double ot) {
    outtakeTime = ot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * exports data to Shuffleboard
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Amp");
    builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
    builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
    //builder.addDoubleProperty("Time to Prime", this::getPrimeTime, this::setPrimeTime);
    //builder.addDoubleProperty("Time to Outtake", this::getOuttakeTime, this::setOuttakeTime);
    builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
    builder.addBooleanProperty("Pneumatic Status", this::getPneumaticState, null);
  }

}
