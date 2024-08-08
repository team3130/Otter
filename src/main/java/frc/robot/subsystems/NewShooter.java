// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static java.lang.Math.abs;

public class NewShooter extends SubsystemBase {
  private final TalonFX topShooterBar;
  private final TalonFX bottomShooterBar;
  private double shooterVolts = 5;
  private double shooterToAmpVolts = 1;
  private final DigitalInput shooterBeam;
  private boolean shooterReachedSpeed = false;
  private final VoltageOut voltRequestTopBar = new VoltageOut(0);
  private final VoltageOut voltRequestBottomBar = new VoltageOut(0);
  private final VelocityVoltage topVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0).withSlot(1);
  private CurrentLimitsConfigs topCurrentConfigs = new CurrentLimitsConfigs();
  private CurrentLimitsConfigs bottomCurrentConfigs = new CurrentLimitsConfigs();

  private double topTargetVelocity = 35;
  private double bottomTargetVelocity = 35;

  private double topTargetShuttleVelocity = 20;
  private double bottomTargetShuttleVelocity = 20;

  private boolean tryingToShoot = false;
  private boolean tryingToShuttle = false;

  private double maxTime = 0.5;



  Slot0Configs slot0Configs; //holds all the pid values (gains) for top flywheel
  private double slot0_kS = 0.0;
  private double slot0_kV = 0.12;
  private double slot0_kP = 0.01;
  private double slot0_kI = 0.0;
  private double slot0_kD = 0.0;

  Slot1Configs slot1Configs; //bottom flywheel gains
  private double slot1_kS = 0.0;
  private double slot1_kV = 0.117;
  private double slot1_kP = 0.01;
  private double slot1_kI = 0.0;
  private double slot1_kD = 0.0;

  public NewShooter() {
    topShooterBar = new TalonFX(Constants.CAN.shooterTopFlywheel);
    bottomShooterBar = new TalonFX(Constants.CAN.shooterBottomFlywheel);
    shooterBeam = new DigitalInput(Constants.IDs.shooterBeamDIO);

    topShooterBar.getConfigurator().apply(new TalonFXConfiguration());
    bottomShooterBar.getConfigurator().apply(new TalonFXConfiguration());

    topShooterBar.setNeutralMode(NeutralModeValue.Coast);
    bottomShooterBar.setNeutralMode(NeutralModeValue.Coast);

    topShooterBar.setInverted(true);
    bottomShooterBar.setInverted(true);

    topShooterBar.getConfigurator().apply(topCurrentConfigs.withSupplyCurrentLimit(40));
    bottomShooterBar.getConfigurator().apply(bottomCurrentConfigs.withSupplyCurrentLimit(40));

    slot0Configs = new Slot0Configs();
    slot1Configs = new Slot1Configs();

    slot0Configs.kS = slot0_kS;
    slot0Configs.kV = slot0_kV;
    slot0Configs.kP = slot0_kP;
    slot0Configs.kI = slot0_kI;
    slot0Configs.kD = slot0_kD;

    slot1Configs.kS = slot1_kS;
    slot1Configs.kV = slot1_kV;
    slot1Configs.kP = slot1_kP;
    slot1Configs.kI = slot1_kI;
    slot1Configs.kD = slot1_kD;

    topShooterBar.getConfigurator().apply(slot0Configs);
    bottomShooterBar.getConfigurator().apply(slot1Configs);
  }

  public void runShooter(){
    topShooterBar.setControl(voltRequestTopBar.withOutput(shooterVolts));
    bottomShooterBar.setControl(voltRequestBottomBar.withOutput(shooterVolts));
  }
  public void runShooterToAmp(){
    topShooterBar.setControl(voltRequestTopBar.withOutput(shooterToAmpVolts));
    topShooterBar.setControl(voltRequestBottomBar.withOutput(shooterToAmpVolts));
  }
  public void stopShooter(){
    topShooterBar.setControl(voltRequestTopBar.withOutput(0));
    bottomShooterBar.setControl(voltRequestBottomBar.withOutput(0));
  }
  public void configureShooterConfigs(){
    topShooterBar.getConfigurator().apply(slot0Configs);
    bottomShooterBar.getConfigurator().apply(slot1Configs);
  }
  public void updateShooterPID(){
    slot0Configs.kS = slot0_kS;
    slot0Configs.kV = slot0_kV;
    slot0Configs.kP = slot0_kP;
    slot0Configs.kI = slot0_kI;
    slot0Configs.kD = slot0_kD;

    slot1Configs.kS = slot1_kS;
    slot1Configs.kV = slot1_kV;
    slot1Configs.kP = slot1_kP;
    slot1Configs.kI = slot1_kI;
    slot1Configs.kD = slot1_kD;
  }
  public void checkShooterAtSetpoint(){
    if (isTryingToShoot()){
      boolean reached = Math.abs(topTargetVelocity - getTopShooterVelocity()) < 2 && Math.abs(bottomTargetVelocity - getBottomShooterVelocity()) < 2;
      setShooterReachedSpeed(reached);
    }
    else if(isTryingToShuttle()){
      boolean reached = Math.abs(topTargetShuttleVelocity - getTopShooterVelocity()) < 2 && Math.abs(bottomTargetShuttleVelocity - getBottomShooterVelocity()) < 2;
      setShooterReachedSpeed(reached);
    }
  }
  public void setShooterVelocity(){
    topShooterBar.setControl(topVelocityRequest.withVelocity(topTargetVelocity));
    bottomShooterBar.setControl(bottomVelocityRequest.withVelocity(bottomTargetVelocity));
  }
  public void setShuttleShooterVelocity(){
    topShooterBar.setControl(topVelocityRequest.withVelocity(topTargetShuttleVelocity));
    topShooterBar.setControl(bottomVelocityRequest.withVelocity(bottomTargetShuttleVelocity));
  }
  public void setTopShooterVelocity(double value){
    topShooterBar.setControl(topVelocityRequest.withVelocity(value));
  }
  public double getTopShooterVelocity(){
    return topShooterBar.getVelocity().getValue();
  }
  public void setBottomShooterVelocity(double value){
    bottomShooterBar.setControl(bottomVelocityRequest.withVelocity(value));
  }
  public double getBottomShooterVelocity(){
    return bottomShooterBar.getVelocity().getValue();
  }
  public void setShooterWithMomentum(double currentTime){ //this is shoot with moving setpoint
    double timePercent = currentTime/maxTime; //percentage of time
    if (timePercent < 1){ //check to make sure you are not using more than 100% of setpoint
      double topShooterRemaining = topTargetVelocity - getTopShooterVelocity(); //way to go from starting to target velocity
      double bottomShooterRemaining = bottomTargetVelocity - getBottomShooterVelocity();
      setTopShooterVelocity(getTopShooterVelocity() + (timePercent * topShooterRemaining)); //makes setpoints gradually increase as timer increases
      setBottomShooterVelocity(getBottomShooterVelocity() + (timePercent * bottomShooterRemaining)); //bottom and top are different in case of different starting points
    }
    else {
      setShooterVelocity();
    }
  }

  //getters
  public boolean getShooterBeam(){return !shooterBeam.get();}
  public double getSlot0_kS(){return slot0_kS;}
  public double getSlot0_kV(){return slot0_kV;}
  public double getSlot0_kP(){return slot0_kP;}
  public double getSlot0_kI(){return slot0_kI;}
  public double getSlot0_kD(){return slot0_kD;}
  public double getSlot1_kS(){return slot1_kS;}
  public double getSlot1_kV(){return slot1_kV;}
  public double getSlot1_kP(){return slot1_kP;}
  public double getSlot1_kI(){return slot1_kI;}
  public double getSlot1_kD(){return slot1_kD;}
  public double getMaxTime(){return maxTime;}
  public boolean hasShooterReachedSpeed(){return shooterReachedSpeed;}
  public boolean isTryingToShoot(){return tryingToShoot;}
  public boolean isTryingToShuttle(){return tryingToShuttle;}
  public double getShooterVolts(){return shooterVolts;}
  public double getShooterToAmpVolts(){return shooterToAmpVolts;}
  public double getTopTargetVelocity(){return topTargetVelocity;}
  public double getBottomTargetVelocity(){return bottomTargetVelocity;}
  public double getTopTargetShuttleVelocity(){return topTargetShuttleVelocity;}
  public double getBottomTargetShuttleVelocity(){return bottomTargetShuttleVelocity;}

  //setters
  public void setSlot0_kS(double value){
    slot0_kS = value;
  }
  public void setSlot0_kV(double value){
    slot0_kV = value;
  }
  public void setSlot0_kP(double value){
    slot0_kP = value;
  }
  public void setSlot0_kI(double value){
    slot0_kI = value;
  }
  public void setSlot0_kD(double value){
    slot0_kD = value;
  }
  public void setSlot1_kS(double value){
    slot1_kS = value;
  }
  public void setSlot1_kV(double value){
    slot1_kV = value;
  }
  public void setSlot1_kP(double value){
    slot1_kP = value;
  }
  public void setSlot1_kI(double value){
    slot1_kI = value;
  }
  public void setSlot1_kD(double value){
    slot1_kD = value;
  }
  public void setMaxTime(double value){
    maxTime = value;
  }
  public void setShooterReachedSpeed(boolean value){
    shooterReachedSpeed = value;
  }
  public void setTryingToShoot(boolean value){
    tryingToShoot = value;
  }
  public void setTryingToShuttle(boolean value){
    tryingToShuttle = value;
  }
  public void setShooterVolts(double value){
    shooterVolts = value;
  }
  public void setShooterToAmpVolts(double value){
    shooterToAmpVolts = value;
  }
  public void setTopTargetVelocity(double value){
    topTargetVelocity = value;
  }
  public void setBottomTargetVelocity(double value){
    bottomTargetVelocity = value;
  }
  public void setTopTargetShuttleVelocity(double value){
    topTargetShuttleVelocity = value;
  }
  public void setBottomTargetShuttleVelocity(double value){
    bottomTargetShuttleVelocity = value;
  }
  public void initSendable(SendableBuilder builder){
    if (Constants.debugMode){
      builder.setSmartDashboardType("New Shooter");

      builder.addBooleanProperty("Shooter Beam", this::getShooterBeam, null);
      builder.addBooleanProperty("Has Reached Speed", this::hasShooterReachedSpeed, this::setShooterReachedSpeed);
      builder.addBooleanProperty("Trying to Shoot", this::isTryingToShoot, this::setTryingToShoot);
      builder.addBooleanProperty("Trying to Shuttle", this::isTryingToShuttle, this::setTryingToShuttle);

      builder.addDoubleProperty("Shooter Volts", this::getShooterVolts, this::setShooterVolts);
      builder.addDoubleProperty("Shooter to Amp Volts", this::getShooterToAmpVolts, this::setShooterToAmpVolts);
      builder.addDoubleProperty("Top Target Velocity", this::getTopTargetVelocity, this::setTopTargetVelocity);
      builder.addDoubleProperty("Bottom Target Velocity", this::getBottomTargetVelocity, this::setBottomTargetVelocity);
      builder.addDoubleProperty("Top Shuttling Target Velocity", this::getTopTargetShuttleVelocity, this::setTopTargetShuttleVelocity);
      builder.addDoubleProperty("Bottom Shuttling target Velocity", this::getBottomTargetShuttleVelocity, this::setBottomTargetShuttleVelocity);
      builder.addDoubleProperty("Top Shooter Velocity", this::getTopShooterVelocity, null);
      builder.addDoubleProperty("Bottom Shooter Velocity", this::getBottomShooterVelocity, null);

      builder.addDoubleProperty("Top S", this::getSlot0_kS, this::setSlot0_kS);
      builder.addDoubleProperty("Top V", this::getSlot0_kV, this::setSlot0_kV);
      builder.addDoubleProperty("Top P", this::getSlot0_kP, this::setSlot0_kP);
      builder.addDoubleProperty("Top I", this::getSlot0_kI, this::setSlot0_kI);
      builder.addDoubleProperty("Top D", this::getSlot0_kD, this::setSlot0_kD);

      builder.addDoubleProperty("Bottom S", this::getSlot1_kS, this::setSlot1_kS);
      builder.addDoubleProperty("Bottom V", this::getSlot1_kV, this::setSlot1_kV);
      builder.addDoubleProperty("Bottom P", this::getSlot1_kP, this::setSlot1_kP);
      builder.addDoubleProperty("Bottom I", this::getSlot1_kI, this::setSlot1_kI);
      builder.addDoubleProperty("Bottom D", this::getSlot1_kD, this::setSlot1_kD);
    }
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

