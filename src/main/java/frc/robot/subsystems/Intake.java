// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PNM_INTAKE_ACTUATOR;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX intakemotor;
  private final Solenoid intakesolenoid1;
  private final Solenoid intakesolenoid2;

  private final DigitalInput limitSwitch1;

  public static double dumbSpeed = .8;

  public Intake() {
    intakemotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    intakesolenoid1 = new Solenoid(Constants.CAN.intakesolenoid1, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    intakesolenoid2 = new Solenoid(Constants.CAN.intakesolenoid2, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    limitSwitch1 = new DigitalInput(Constants.CAN.intakeLimitSwitch1);



    intakemotor.setInverted(false);


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
  public double getDumbSpeed(){
    return dumbSpeed;
  }
  public void setDumbSpeed(double x){
    dumbSpeed = x;
  }
  public boolean intakeLimitSwitch1(){
    return limitSwitch1.get();
  }
  public void DumbIntake(){
    intakemotor.set(dumbSpeed);
  }

  public void DumbOuttake(){
    intakemotor.set(-dumbSpeed);
  }

  public void Stoptake(){
    intakemotor.set(0);
  }

  public void SolenoidToggle(){
    intakesolenoid1.toggle();
    intakesolenoid2.toggle();
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
