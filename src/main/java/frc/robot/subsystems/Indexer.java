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
  private double indexerVoltageCompensation = 10;
  private double outakeSpeed = -1;
  private double spintakeSpeed = 1; // 10
  private double shooterSpindexSpeed = 0.3; // 3 volts
  private double autoSpintakeSpeed = 1; // 10
  private double autoShooterSpindexSpeed = 1;

  public Indexer() {
    indexer = new WPI_TalonSRX(Constants.CAN.intakeIndexer);

    indexer.configFactoryDefault();
    indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    indexer.setNeutralMode(NeutralMode.Coast);
    indexer.configVoltageCompSaturation(indexerVoltageCompensation);
    indexer.enableVoltageCompensation(true);

    indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    indexer.setInverted(false);
  }

  public void spintake() {
    indexer.set(spintakeSpeed);
  }

  public void outtake(){
    indexer.set(outakeSpeed);
  }

  public void shooterSpindex() {
    indexer.set(shooterSpindexSpeed);
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
  public void setShooterSpindexSpeed(double lol) { shooterSpindexSpeed = lol;}
  public double getShooterSpindexSpeed() { return shooterSpindexSpeed; }

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Indexer");
      builder.addDoubleProperty("Spintake speed", this::getSpintakeSpeed, this::setSpintakeSpeed);
      builder.addDoubleProperty("Outtake speed", this::getOutakeSpeed, this::setOutakeSpeed);
      builder.addDoubleProperty("Shooter Index speed", this::getShooterSpindexSpeed, this::setShooterSpindexSpeed);
      builder.addDoubleProperty("Indexer Voltage comp", this::getIndexerVoltageCompensation, this::setIndexerVoltageCompensation);
    }
  }

}
