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
  private final Solenoid shifterOne; // shorter pneumatic
  private final Solenoid shifterTwo; // longer pneumatic
  private boolean doubleRetracted = true; // both pneumatics down
  private boolean shifterOneExtended = false; // pneumatic 1 up
  private boolean shifterTwoExtended = false; // pneumatic 2 up
  private boolean doubleExtended = false; // both pneumatics up

  public ShooterShifter() {
    shifterOne = new Solenoid(Constants.CAN.shifterOnePNM, PneumaticsModuleType.CTREPCM , Constants.CAN.shifterOneChannel);
    shifterTwo = new Solenoid(Constants.CAN.shifterTwoPNM, PneumaticsModuleType.CTREPCM , Constants.CAN.shifterTwoChannel);

    shifterOne.set(false);
    shifterTwo.set(false);
  }

  public boolean getIsParked() { return doubleRetracted; }
  public boolean getIsFirstShootStage() { return shifterOneExtended; }
  public boolean getIsSecondShootStage() { return shifterTwoExtended; }
  public boolean getIsThirdShootStage() { return doubleExtended; }
  public void setDoubleRetract(boolean newIsParked) { doubleRetracted = newIsParked; }
  public void extendShifterOne(boolean newIsFirstShootStage) { shifterOneExtended = newIsFirstShootStage; }
  public void extendShifterTwo(boolean newIsSecondShootStage) { shifterTwoExtended = newIsSecondShootStage; }
  public void setDoubleExtended(boolean newIsThirdShootStage) { shifterTwoExtended = newIsThirdShootStage; }

  /**
  METHODS
   */
  public void goToParkedStage(){
    setDoubleRetract(true);
    shifterOne.set(false);
    shifterTwo.set(false);
  }
  public void goToStage1(){
    extendShifterOne(true);
    shifterOne.set(true);
    shifterTwo.set(false);
  }
  public void goToStage2(){
    extendShifterTwo(true);
    shifterOne.set(false);
    shifterTwo.set(true);
  }
  public void goToStage3(){
    setDoubleExtended(true);
    shifterOne.set(true);
    shifterTwo.set(true);
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
