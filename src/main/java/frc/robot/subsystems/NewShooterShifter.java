// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewShooterShifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Solenoid shortShifter;
  private final Solenoid longShifter;
  private boolean lowPosition = true;
  private boolean shortPosition = false;
  private boolean longPosition = false;
  private boolean highPosition = false;
  public NewShooterShifter() {
    shortShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    longShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  }

  public void goLowPosition(){
    setShortShifter(false);
    setLongShifter(false);

    setLowPosition(true);
    setShortPosition(false);
    setLongPosition(false);
    setHighPosition(false);
  }
  public void goShortPosition(){
    setShortShifter(true);
    setLongShifter(false);

    setLowPosition(false);
    setShortPosition(true);
    setLongPosition(false);
    setHighPosition(false);
  }
  public void goLongPosition(){
    setShortShifter(false);
    setLongShifter(true);

    setLowPosition(false);
    setShortPosition(false);
    setLongPosition(true);
    setHighPosition(false);
  }
  public void goHighPosition(){
    setShortShifter(true);
    setLongShifter(true);

    setLowPosition(false);
    setShortPosition(false);
    setLongPosition(false);
    setHighPosition(true);
  }

  //Getters
  public boolean getLowPosition(){return lowPosition;}
  public boolean getShortPosition(){return shortPosition;}
  public boolean getLongPosition(){return longPosition;}
  public boolean getHighPosition(){return highPosition;}
  
  //Setters
  public void setShortShifter(boolean value){
    shortShifter.set(value);
  }
  public void setLongShifter(boolean value){
    longShifter.set(value);
  }
  public void setLowPosition(boolean value){
    lowPosition = value;
  }
  public void setShortPosition(boolean value){
    shortPosition = value;
  }
  public void setLongPosition(boolean value){
    longPosition = value;
  }
  public void setHighPosition(boolean value){
    highPosition = value;
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
