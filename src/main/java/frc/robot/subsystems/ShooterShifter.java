// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterShifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  /**
  VARIABLES
   */
  // solenoid1 is shorter pneumatic
  private final Solenoid solenoid1;

  // solenoid2 is longer pneumatic
  private final Solenoid solenoid2;

  /**
   * is parked is a variable for when both solenoids are not activated
   */
  private boolean isParked = true;

  private boolean isFirstShootStage = false;
  private boolean isSecondShootStage = false;
  private boolean isThirdShootStage = false;

  public ShooterShifter() {
    /**
     * fill in parameters with pneumatic type
     */
    solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.CAN.CAN_Solenoid1);
    solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.CAN.CAN_Solenoid2);

    solenoid1.set(false);
    solenoid2.set(false);
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
  public void goToParkedStage(){
    setIsParked(true);
    solenoid1.set(false);
    solenoid2.set(false);
  }
  public void goToStage1(){
    setFirstShootStage(true);
    solenoid1.set(true);
    solenoid2.set(false);
  }
  public void goToStage2(){
    setSecondShootStage(true);
    solenoid1.set(false);
    solenoid2.set(true);
  }
  public void goToStage3(){
    setThirdShootStage(true);
    solenoid1.set(true);
    solenoid2.set(true);
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
