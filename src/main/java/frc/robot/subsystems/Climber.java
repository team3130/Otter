// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final DigitalInput m_limitSwitchR;
  private final DigitalInput m_limitSwitchL;

  private final WPI_TalonSRX m_motorR;
  private final WPI_TalonSRX m_motorL;

  public Climber() {
    m_motorR = new WPI_TalonSRX(Constants.Climber.KRMotor);
    m_motorR.configFactoryDefault();
    m_motorR.setInverted(false);

    m_motorL = new WPI_TalonSRX(Constants.Climber.KLMotor);
    m_motorL.configFactoryDefault();
    m_motorL.setInverted(false);

    m_limitSwitchR = new DigitalInput(Constants.Climber.kRLimitSwitch);
    m_limitSwitchL = new DigitalInput(Constants.Climber.kRLimitSwitch);
  }

  public boolean brokeLeft() {
    return !m_limitSwitchL.get();
  }

  public boolean brokeRight() {
    return !m_limitSwitchR.get();
  }

  public void setSpeedRight(double speed) {
    m_motorR.set(ControlMode.PercentOutput, speed);
  }

  public void setSpeedLeft(double speed) {
    m_motorL.set(ControlMode.PercentOutput, speed);
  }

  public void stopRight() {
    m_motorR.set(ControlMode.PercentOutput, 0);
  }

  public void stopLeft() {
    m_motorL.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addBooleanProperty("ClimberBrokeRight", this::brokeRight, null);
      builder.addBooleanProperty("ClimberBrokeLeft", this::brokeLeft, null);
  }
}