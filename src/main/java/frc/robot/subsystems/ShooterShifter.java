// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterShifter extends SubsystemBase {
  private final Solenoid shortShifter; // shorter pneumatic
  private final Solenoid longShifter; // longer pneumatic
  private boolean doubleRetracted = true; // both pneumatics down
  private boolean shifterOneExtended = false; // pneumatic 1 up
  private boolean shifterTwoExtended = false; // pneumatic 2 up
  private boolean doubleExtended = false; // both pneumatics up

  public ShooterShifter() {
    shortShifter = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM , Constants.IDs.longShifterChannel);
    longShifter = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM , Constants.IDs.smallShifterChannel);

    shortShifter.set(false);
    longShifter.set(false);
  }

  public void doubleRetract() {
    shortShifter.set(false);
    longShifter.set(false);
    setDoubleRetract(true);
  }

  public void extendShortShifter() {
    shortShifter.set(true);
    longShifter.set(false);
    setShortShifter(true);
  }

  public void extendLongShifter() {
    shortShifter.set(false);
    longShifter.set(true);
    setLongShifter(true);
  }

  public void doubleExtend() {
    shortShifter.set(true);
    longShifter.set(true);
    setDoubleExtended(true);
  }

  public boolean getIsParked() { return doubleRetracted; }
  public boolean getIsFirstShootStage() { return shifterOneExtended; }
  public boolean getIsSecondShootStage() { return shifterTwoExtended; }
  public boolean getIsThirdShootStage() { return doubleExtended; }
  public void setDoubleRetract(boolean newIsParked) { doubleRetracted = newIsParked; }
  public void setShortShifter(boolean newIsFirstShootStage) { shifterOneExtended = newIsFirstShootStage; }
  public void setLongShifter(boolean newIsSecondShootStage) { shifterTwoExtended = newIsSecondShootStage; }
  public void setDoubleExtended(boolean newIsThirdShootStage) { shifterTwoExtended = newIsThirdShootStage; }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Shooter Shifter");
      builder.addBooleanProperty("speed", this::getIsParked, null);
      builder.addBooleanProperty("speed", this::getIsFirstShootStage, null);
      builder.addBooleanProperty("speed", this::getIsSecondShootStage, null);
      builder.addBooleanProperty("speed", this::getIsThirdShootStage, null);
    }
  }
}
