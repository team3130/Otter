// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterShifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  /**
  VARIABLES
   */
  private final Solenoid solenoid1;
  /**
   * solenoid1 is shorter pneumatic
   */
  private final Solenoid solenoid2;
  /**
   * solenoid2 is longer pneumatic
   */
  private boolean isParked = true;
  /**
   * is parked is a variable for when both solenoids are not activated
   */
  private boolean isFirstShootStage = true;
  private boolean isSecondShootStage = true;
  private boolean isThirdShootStage = true;
  public ShooterShifter() {
    /**
     * fill in parameters with canID and pneumatic type
     */
    solenoid1 = new Solenoid();
    solenoid2 = new Solenoid();

  }

  /**
   * GETTERS
   */
  public boolean getIsParked(){
    return isParked;
  }
  public boolean getIsFirstShootStage(){
    return isFirstShootStage;
  }
  public boolean getIsSecondShootStage(){return isSecondShootStage;}
  public boolean getIsThirdShootStage(){return isThirdShootStage;}

  /**
   * SETTERS
   */
  public void setIsParked(boolean newIsParked){
    isParked = newIsParked;
  }
  public void setFirstShootStage(boolean newIsFirstShootStage){
    isFirstShootStage = newIsFirstShootStage;
  }
  public void setSecondShootStage(boolean newIsSecondShootStage){isSecondShootStage = newIsSecondShootStage;}
  public void setThirdShootStage(boolean newIsThirdShootStage){isSecondShootStage = newIsThirdShootStage;}

  /**
  METHODS
   */
  public void increaseStage(){
    /**
     * this is the shooter shifter stage 1 from parked stage
     */
    if(isParked) {
      solenoid1.set(true);
      setFirstShootStage(true);
      setIsParked(false);
      setSecondShootStage(false);
      setThirdShootStage(false);
    }
    /**
     * this is the shooter shifter stage 2 from stage 1
     */
    if(isFirstShootStage){
      solenoid1.set(false);
      solenoid2.set(true);
      setSecondShootStage(true);
      setFirstShootStage(false);
      setThirdShootStage(false);
      setIsParked(false);
    }
    /**
     * this is the shooter shifter stage 3 from stage 2
     */
    if(isSecondShootStage){
      solenoid1.set(true);
      solenoid2.set(true);
      setThirdShootStage(true);
      setIsParked(false);
      setFirstShootStage(false);
      setSecondShootStage(false);
    }
  }
  public void decreaseStage(){
    /**
     * this is the shooter shifter stage 2 from stage 3
     */
    if(isThirdShootStage){
      solenoid1.set(false);
      solenoid2.set(true);
      setSecondShootStage(true);
      setThirdShootStage(false);
      setFirstShootStage(false);
      setIsParked(false);
    }
    /**
     * this is the shooter shifter stage 1 from stage 2
     */
    if(isSecondShootStage){
      solenoid1.set(true);
      solenoid2.set(false);
      setFirstShootStage(true);
      setThirdShootStage(false);
      setSecondShootStage(false);
      setIsParked(false);
    }
    /**
     * this is the shooter shifter parked stage from stage 1
     */
    if(isFirstShootStage){
      solenoid1.set(false);
      solenoid2.set(false);
      setIsParked(true);
      setThirdShootStage(false);
      setSecondShootStage(false);
      setFirstShootStage(false);
    }
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
