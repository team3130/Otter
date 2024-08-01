// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewIndexer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final NewShooter shooter;
  private final WPI_TalonSRX indexer;
  private double indexerVoltageCompensation = 12;
  private double intakeSpeed = 1;

  public NewIndexer(NewShooter shooter) {
    this.shooter = shooter;

    indexer = new WPI_TalonSRX(Constants.CAN.intakeIndexer); //defining indexer

    indexer.configFactoryDefault(); //setting motor to default settings
    indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder); //selecting quad encoder\
    indexer.setNeutralMode(NeutralMode.Coast);//when no input is sent it will be coasting
    indexer.enableVoltageCompensation(true);
    indexer.configVoltageCompSaturation(indexerVoltageCompensation);
    indexer.setInverted(false); //idk if i need to do this since false is default i think
  }

  //Getters
  public double getIntakeSpeed(){ return intakeSpeed; }
  public double getIndexerVoltageCompensation(){ return indexerVoltageCompensation; }
  public double getIndexerVoltage(){ return indexer.getSupplyCurrent(); }

  //Setters
  public void setIntakeSpeed(double newSpeed){
    intakeSpeed = newSpeed;
  }
  public void setIndexerVoltageCompensation(double value){
    indexerVoltageCompensation = value;
  }


  public void spinIntake(){
    indexer.set(intakeSpeed);
  }
  public void spinOuttake(){
    indexer.set(-intakeSpeed);
  }
  public void stopIntake(){
    indexer.set(0);
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
