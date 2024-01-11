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

  public boolean brokeLimitR() {
    return !m_limitSwitchR.get();
  }

  public boolean brokeLimitL() {
    return !m_limitSwitchL.get();
  }

  public void setSpeedR() {
    m_motorR.set(0); // TODO
  }

  public void setSpeedL() {
    m_motorL.set(0); // TODO
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
