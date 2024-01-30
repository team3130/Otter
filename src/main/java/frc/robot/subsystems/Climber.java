// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final DigitalInput m_limitSwitchR;
  private final DigitalInput m_limitSwitchL;

  private final WPI_TalonFX m_motorR;
  private final WPI_TalonFX m_motorL;

  private boolean isZeroed = false;

  public static double maxExtensionTicks = 18300; // TODO
  public static double kExtensionDeadband = 0.1; //The % of max extension where it will slow down (works on both ends)
  public static double percentage = 15;
  public static double extensionTicksToArmDistance = 0; // TODO // conversion factor from ticks to distance of arm extension
  public static double extensionFactorScalar = slowExtensionEndsDistance; // TODO

  public Climber() {
    m_motorR = new WPI_TalonFX(Constants.Climber.KRMotor);
    m_motorR.configFactoryDefault();
    m_motorR.setInverted(false);

    m_motorL = new WPI_TalonFX(Constants.Climber.KLMotor);
    m_motorL.configFactoryDefault();
    m_motorL.setInverted(false);

    m_limitSwitchR = new DigitalInput(Constants.Climber.KRLimitSwitch);
    m_limitSwitchL = new DigitalInput(Constants.Climber.KRLimitSwitch);
  }

  public boolean brokeLeft() {
    return !m_limitSwitchL.get();
  }

  public boolean brokeRight() {
    return !m_limitSwitchR.get();
  }

  public void setSpeedRight(double speed) {
    m_motorR.set(speed);
  }

  public void setSpeedLeft(double speed) {
    m_motorL.set(speed);
  }

  public void stop() {
    m_motorR.set(ControlMode.PercentOutput, 0);
    m_motorL.set(ControlMode.PercentOutput, 0);
  }

  public void stopRight() {
    m_motorR.set(ControlMode.PercentOutput, 0);
  }

  public void stopLeft() {
    m_motorL.set(ControlMode.PercentOutput, 0);
  }

  /**zeroes encoders*/
  public void resetEncoders() {
    m_motorL.setSelectedSensorPosition(0);
    m_motorR.setSelectedSensorPosition(0);
  }

  public void resetEncodersLeft() {
    m_motorL.setSelectedSensorPosition(0);
  }

  public void resetEncodersRight() {
    m_motorR.setSelectedSensorPosition(0);
  }

  public void setZeroed(boolean value) {
    isZeroed = value;
  }

  public void zero(double left, double right) {
    m_left_motor.set(ControlMode.PercentOutput, -left);
     m_right_motor.set(ControlMode.PercentOutput, -right);
}

  public boolean isZeroed() {
    return isZeroed;
  }

  public double getMaxExtensionTicks(){
    return maxExtensionTicks;
  }

  public void setMaxExtensionTicks(double x){
    maxExtensionTicks = x;
  }

  public double getkExtensionDeadband(){
    return kExtensionDeadband;
  }

  public void setkExtensionDeadband(double x){
    kExtensionDeadband = x;
  }

  public double getExtensionTicksToArmDistance(){
    return extensionTicksToArmDistance;
  }

  public void setExtensionTicksToArmDistance(double x){
    extensionTicksToArmDistance = x;
  }

  public double getExtensionFactorScalar(){
    return extensionFactorScalar;
  }

  public void setExtensionFactorScalar(double x){
    extensionFactorScalar = x;
  }

  public double getPercentage() {
    return percentage;
  }

  public double getJoystickValue() {
    return -RobotContainer.m_WeaponsGamepad.getRawAxis(1);
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