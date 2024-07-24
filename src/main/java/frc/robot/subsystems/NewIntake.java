// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Solenoid intakeSolenoid;
  private final DigitalInput intakeLimit;
  public NewIntake() {
    intakeSolenoid = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.intakePNMChannel);
    intakeLimit = new DigitalInput(Constants.IDs.intakeLimitDIO);
  }

  public boolean getIntakeLimit(){
    return intakeLimit.get();
  }

  public void intakeOut(){
    intakeSolenoid.set(true);
  }

  public void intakeIn(){
    intakeSolenoid.set(false);
  }

  public void toggleIntake(){
    intakeSolenoid.toggle();
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
