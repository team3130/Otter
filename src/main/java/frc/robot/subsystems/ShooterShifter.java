// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterShifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //VARIABLES
  private final Solenoid solenoid1;
  private boolean isParked = true;
  private final Solenoid solenoid2;
  private boolean isFirstShootStage = true;
  private boolean is
  public ShooterShifter() {
    // fill in parameters with canID and pneumatic type
    solenoid1 = new Solenoid();
    solenoid2 = new Solenoid();

  }

  //GETTERS
  public boolean getIsParked(){
    return isParked;
  }
  public boolean getIsFirstShootStage(){
    return isFirstShootStage;
  }

  //SETTERS
  public void setIsParked(boolean newIsParked){
    isParked = newIsParked;
  }
  public void setFirstShootStage(boolean newIsFirstShootStage){
    isFirstShootStage = newIsFirstShootStage;
  }

  //METHODS
  public void parkShift(){
    setIsParked(!isParked);
    solenoid1.set(!isParked);
  }
  public void shootShift(){
    setFirstShootStage(!isFirstShootStage);
    solenoid2.set(!isFirstShootStage);
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
