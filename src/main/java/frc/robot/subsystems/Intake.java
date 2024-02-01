// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PNM_INTAKE_ACTUATOR;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX intakemotor;
  private final Solenoid intakesolenoid1;
  private final Solenoid intakesolenoid2;

  private final DigitalInput limitSwitch1;

  public Timer timer;
  public Timer stopwatch;

  public Intake() {
    intakemotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    intakesolenoid1 = new Solenoid(Constants.CAN.intakesolenoid1, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    intakesolenoid2 = new Solenoid(Constants.CAN.intakesolenoid2, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    limitSwitch1 = new DigitalInput(Constants.CAN.intakeLimitSwitch1);

    intakemotor.configFactoryDefault();

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
    return Constants.Intake.dumbSpeed;
  }
  public void setDumbSpeed(double x){
    Constants.Intake.dumbSpeed = x;
  }
  public void slowIntake() {intakemotor.set(Constants.Intake.slowSpeed);}
  public void slowOutake() {intakemotor.set(-Constants.Intake.slowSpeed);}
  public boolean intakeLimitSwitch1(){
    return limitSwitch1.get();
  }
  public void DumbIntake(){
    intakemotor.set(Constants.Intake.dumbSpeed);
  }
  public void DumbOuttake(){
    intakemotor.set(-Constants.Intake.dumbSpeed);
  }
  public boolean limitSwitchTimer(){
    if(!intakeLimitSwitch1()){
      timer.reset();
      timer.start();
    }
    if (timer.hasElapsed(Constants.Intake.allottedTime)) {
      return false;
    } else {
      return true;
    }
  }

  public void Stoptake(){
    intakemotor.set(0);
  }


  public void smartSpin(){
    DumbIntake();
    if (limitSwitchTimer()) {
      if (timer.hasElapsed(Constants.Intake.time1)) {
        slowIntake();
      }
      if (timer.hasElapsed(Constants.Intake.time2)) {
        Stoptake();
      }
    else {
      DumbIntake();
    }
    }
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
