
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import static java.lang.Math.abs;


public class Amp extends SubsystemBase {
  private final DigitalInput ampLimit;
  private final WPI_TalonSRX ampLiftingMotor;
  private final WPI_TalonSRX ampSpinningMotor;
  private double intakeAmpSpeed = 1;
  private double ampLiftSpeed = 0.5;
  private double ampLowerSpeed = -0.5;
  private double outtakeAmpSpeed = -1;
  private int encoderMaxTicks = 13600;
  private int highSetpoint = 13500; //drop into amp
  private int midSetpoint = 5465; //pick up from mid shooter
  private int lowSetpoint = 0;
  private PIDController ampController;
  private double P = 0.0013;
  private double I = 0;
  private double D = 0.00005;
  private boolean hasZeroed = false;
  private boolean isMid = false;
  private boolean isHigh = false;
  private boolean isReadyToScore = false;

  public Amp() {
    ampLimit = new DigitalInput(Constants.IDs.ampLimitDIO);

    ampLiftingMotor = new WPI_TalonSRX(Constants.CAN.ampLiftMotor);
    ampSpinningMotor = new WPI_TalonSRX(Constants.CAN.ampSpinMotor);

    ampLiftingMotor.configFactoryDefault();
    ampSpinningMotor.configFactoryDefault();

    ampSpinningMotor.setNeutralMode(NeutralMode.Brake);
    ampLiftingMotor.setNeutralMode(NeutralMode.Brake);

    ampSpinningMotor.enableVoltageCompensation(true);
    ampSpinningMotor.configVoltageCompSaturation(9);

    ampLiftingMotor.enableVoltageCompensation(true);
    ampLiftingMotor.configVoltageCompSaturation(5);

    ampLiftingMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    ampLiftingMotor.setInverted(true);
    ampSpinningMotor.setInverted(true);

    ampSpinningMotor.enableCurrentLimit(true);
    ampSpinningMotor.configContinuousCurrentLimit(20);
    ampSpinningMotor.configPeakCurrentLimit(0);

    ampLiftingMotor.enableCurrentLimit(true);
    ampLiftingMotor.configContinuousCurrentLimit(20);
    ampLiftingMotor.configPeakCurrentLimit(0);


    ampController = new PIDController(P, I, D);
  }

  public boolean getIsReadyToScore() {
    return isReadyToScore;
  }

  public void setIsReadyToScore(boolean lol) {
    isReadyToScore = lol;
  }

  public boolean getHasZeroed(){
    return hasZeroed;
  }
  public void setHasZeroedTrue(){
    hasZeroed = true;
  }

  public void resetController(){
    ampController.setTolerance(100);
    ampController.setPID(P, I, D);
  }

  public int getHighSetpoint(){
    return highSetpoint;
  }
  public int getMidSetpoint() { return midSetpoint; }
  public int getLowSetpoint(){
    return lowSetpoint;
  }


  public void setHighSetpoint(long set){
    highSetpoint = (int) set;
  }
  public void setMidSetpoint(long set){
    midSetpoint = (int) set;
  }
  public void setLowSetpoint(long set){
    lowSetpoint = (int) set;
  }



  public double runController( double setpoint){
    return ampController.calculate(getLiftingEncoderPosition(), setpoint);
  }
  public boolean isAtSetpoint(){
    return ampController.atSetpoint();
  }

  public boolean isAtSetpointWithDeadband() {
    return ((abs(ampController.getSetpoint() - getLiftingEncoderPosition())) <= 150);
  }

  public void intakeAmp() {
    ampSpinningMotor.set(ControlMode.PercentOutput, intakeAmpSpeed);
  }
  public void outtakeAmp() {
    ampSpinningMotor.set(ControlMode.PercentOutput, outtakeAmpSpeed);
  }
  public void ampSpinningMotorStop() {
    ampSpinningMotor.set(ControlMode.PercentOutput, 0);
  }
  public void ampLiftingMotorStop() {
    ampLiftingMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetEncoder() {
    ampLiftingMotor.setSelectedSensorPosition(-0);
  }

  public boolean getLimitSwitch() {
    return !ampLimit.get();
  }
  public double getLiftingEncoderPosition() { return -ampLiftingMotor.getSelectedSensorPosition();}

  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }
  public int getEncoderMax(){
    return encoderMaxTicks;
  }
  public void setEncoderMax(long max){
    encoderMaxTicks = (int) max;
  }
  public double getAmpLiftSpeed(){
    return ampLiftSpeed;
  }
  public double getAmpLowerSpeed(){
    return ampLowerSpeed;
  }
  public void setAmpLiftSpeed(double speed){
    ampLiftSpeed = speed;
  }
  public void moveAmpAtSpeed(double speed){
    ampLiftingMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setAmpLowerSpeed(double speed){
    ampLowerSpeed = speed;
  }
  public double getOuttakeAmpSpeed() { return outtakeAmpSpeed; }
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }
  public void setOuttakeAmpSpeed(double speed) {
    outtakeAmpSpeed = speed;
  }

  public void manualAmpLiftUp(){
    ampLiftingMotor.set(ampLiftSpeed);
  }
  public void manualAmpLowerDown(){
    ampLiftingMotor.set(ampLowerSpeed);
  }

  public double getP(){
    return P;
  }
  public double getI(){
    return I;
  }
  public double getD(){
    return D;
  }
  public void setP(double newP){
     P = newP;
  }
  public void setI(double newI){
    I = newI;
  }
  public void setD(double newD){
    D = newD;
  }

  public boolean getIsHigh() { return isHigh;}
  public boolean getIsMid() { return isMid; }
  public void setIsHigh(boolean high) { isHigh = high; }
  public void setIsMid(boolean mid) { isMid = mid; }


  @Override
  public void periodic() {
  }

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Amp");
      builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
      builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
      builder.addDoubleProperty("Lift Amp Speed", this::getAmpLiftSpeed, this::setAmpLiftSpeed);
      builder.addDoubleProperty("Lower Amp Speed", this::getAmpLowerSpeed, this::setAmpLowerSpeed);
      builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
      builder.addIntegerProperty("Encoder Max", this::getEncoderMax, this::setEncoderMax);
      builder.addDoubleProperty("Encoder Position", this::getLiftingEncoderPosition, null);
      builder.addIntegerProperty("high setpoint", this::getHighSetpoint, this::setHighSetpoint);
      builder.addIntegerProperty("low setpoint", this::getLowSetpoint, this::setLowSetpoint);
      builder.addIntegerProperty("mid setpoint", this::getMidSetpoint, this::setMidSetpoint);
      builder.addBooleanProperty("Is At Setpoint", this::isAtSetpoint, null);
      builder.addBooleanProperty("has zeroed", this::getHasZeroed, null);
      builder.addDoubleProperty("p", this::getP, this::setP);
      builder.addDoubleProperty("i", this::getI, this::setI);
      builder.addDoubleProperty("d", this::getD, this::setD);

      builder.addBooleanProperty("is at mid", this::getIsMid, this::setIsMid);
      builder.addBooleanProperty("is at high", this::getIsHigh, this::setIsHigh);
      builder.addBooleanProperty("ready to score", this::getIsReadyToScore, this::setIsReadyToScore);

    }
  }
}