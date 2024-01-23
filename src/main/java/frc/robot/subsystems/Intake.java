// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.Spintake;

import static frc.robot.Constants.PNM_INTAKE_ACTUATOR;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX m_intakemotor;
  private Solenoid m_intakesolenoid;
  private VoltageOut m_request;

  public Intake() {
    m_intakemotor = new TalonFX(Constants.CAN.intakeMotor);
    m_intakesolenoid = new Solenoid(Constants.CAN.intakesolenoid, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);

    m_intakemotor.setInverted(false);

    final VoltageOut m_request = new VoltageOut(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void Intake(){
    m_intakemotor.setControl(m_request.withOutput(12.0));
  }

  public void Outtake(){
    m_intakemotor.setControl(m_request.withOutput(-12.0));
  }

  public void Stoptake(){
    m_intakemotor.setControl(m_request.withOutput(0.0));
  }

  public void SolenoidToggle(){
    m_intakesolenoid.toggle();
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
