// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final WPI_TalonSRX indexer;
  private final Shooter shooter;
  private double indexerVoltageCompensation = 10;
  private double outakeSpeed = -1;
  private double spintakeSpeed = 1; // 10
  private double indexToBeamSpeed = 0.4; // 10

  private double toShooterSpindexSpeed = 0.7;
  private double autoSpintakeSpeed = 1; // 10
  private double autoShooterSpindexSpeed = 1;
  //private SupplyCurrentLimitConfiguration currLimitConfigs = new SupplyCurrentLimitConfiguration();

  public Indexer(Shooter shooter) {
    this.shooter = shooter;
    indexer = new WPI_TalonSRX(Constants.CAN.intakeIndexer);

    indexer.configFactoryDefault();
    indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    indexer.setNeutralMode(NeutralMode.Coast);
    indexer.enableVoltageCompensation(true);
    indexer.configVoltageCompSaturation(indexerVoltageCompensation);

    indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //currLimitConfigs.currentLimit = 30;
    //indexer.configSupplyCurrentLimit(currLimitConfigs);

    indexer.setInverted(false);
  }

  public boolean flywheelVelocitiesReady() {
    if ((shooter.getTopFlyVelocityRPS() > (shooter.getTopVelocitySetpoint() - 2)) && (shooter.getBottomFlyVelocityRPS() > (shooter.getBottomVelocitySetpoint() - 2))) {
      return true;
    } else {
      return false;
    }
  }

  public void spintake() {
    indexer.set(spintakeSpeed);
  }
  public void indexToBeam() {
    indexer.set(indexToBeamSpeed);
  }

  public void outtake(){
    indexer.set(outakeSpeed);
  }

  public void toShooterSpindex() {
    indexer.set(toShooterSpindexSpeed);
  }

  public void stopIndexer() {
    indexer.set(0);
  }

  public void autoSpintake() {
    indexer.set(autoSpintakeSpeed);
  }

  public void autoShooterSpindex() {
    indexer.set(autoShooterSpindexSpeed);
  }

  public void setSpintakeSpeed(double newSpeed) { spintakeSpeed = newSpeed; }
  public double getSpintakeSpeed() { return spintakeSpeed; }
  public void setOutakeSpeed(double newS) { outakeSpeed = newS; }
  public double getOutakeSpeed() { return outakeSpeed; }
  public void setIndexerVoltageCompensation(double volts) { indexerVoltageCompensation = volts;}
  public double getIndexerVoltageCompensation() { return indexerVoltageCompensation; }
  public void setToShooterSpindexSpeed(double lol) { toShooterSpindexSpeed = lol;}
  public double getToShooterSpindexSpeed() { return toShooterSpindexSpeed; }

  public double getIndexerCurrent() { return indexer.getSupplyCurrent();}

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Indexer");
      builder.addDoubleProperty("Spintake speed", this::getSpintakeSpeed, this::setSpintakeSpeed);
      builder.addDoubleProperty("Outtake speed", this::getOutakeSpeed, this::setOutakeSpeed);
      builder.addDoubleProperty("Shooter Index speed", this::getToShooterSpindexSpeed, this::setToShooterSpindexSpeed);
      builder.addDoubleProperty("Indexer Voltage comp", this::getIndexerVoltageCompensation, this::setIndexerVoltageCompensation);
      builder.addDoubleProperty("indexer current", this::getIndexerCurrent, null);
    }
  }

}
