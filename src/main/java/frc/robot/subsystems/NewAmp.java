// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewAmp extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final WPI_TalonSRX trackMotor; //motor for amp going up and down
  private final WPI_TalonSRX wheelMotor; //motor for amp wheels to turn (wheels at end of amp where note is)
  private final DigitalInput ampLimit; //limit switch on bottom of amp track
  private PIDController ampPIDController; //pid controller for amp

  //idk why but pid in shuffleboard and in intellij different so this might not be the best but good enough
  private double P = 0.00022;
  private double I = 0.00000032;
  private double D = 0.00001;

  //speeds
  private final double trackMotorSpeedUp = 0.3;
  private final double trackMotorSpeedDown = -0.2;
  private final double wheelMotorSpeed = 0.5;

  //setpoints
  private final int highSetpoint = 12600;
  private final int midSetpoint = 7600;
  private final int lowSetpoint = 750;

  private boolean ampZeroed = false;
  private final int maxEncoderLimit = 13600;
  private final int minEncoderLimit = 750;
  public NewAmp() {
    //initializing motors to variables
    trackMotor = new WPI_TalonSRX(Constants.CAN.ampLiftMotor);
    wheelMotor = new WPI_TalonSRX(Constants.CAN.ampSpinMotor);

    //factory default for both motors
    trackMotor.configFactoryDefault();
    wheelMotor.configFactoryDefault();

    //sets brake mode (when no input then motors brake)
    trackMotor.setNeutralMode(NeutralMode.Brake);
    wheelMotor.setNeutralMode(NeutralMode.Brake);

    //setting motors inverted
    trackMotor.setInverted(true);
    wheelMotor.setInverted(true);

    //sensors
    trackMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    ampLimit = new DigitalInput(Constants.IDs.ampLimitDIO);

    //pid
    ampPIDController = new PIDController(P,I,D);
  }

  //Wheel Motor basic commands
  public void ampIntake(){
    wheelMotor.set(ControlMode.PercentOutput, wheelMotorSpeed);
  }
  public void ampOuttake(){
    wheelMotor.set(ControlMode.PercentOutput, -wheelMotorSpeed);
  }
  public void wheelMotorStop(){
    wheelMotor.set(ControlMode.PercentOutput, 0);
  }

  //Track Motor basic commands
  public void trackMotorUp(){trackMotor.set(ControlMode.PercentOutput, trackMotorSpeedUp);}
  public void trackMotorDown(){trackMotor.set(ControlMode.PercentOutput, trackMotorSpeedDown);}
  public void trackMotorStop() {trackMotor.set(ControlMode.PercentOutput, 0);}

  public void resetAmpController(){
    ampPIDController.setTolerance(100);
    ampPIDController.setPID(P,I,D);
  }
  public double runAmpController(double setpoint){
    return ampPIDController.calculate(getAmpLocation(), setpoint);
  }
  public void resetAmpEncoder(){
    trackMotor.setSelectedSensorPosition(0);
  }
  public void moveAmpAtSpeed(double speed){ trackMotor.set(ControlMode.PercentOutput, speed); }

  //getters
  public double getTrackMotorSpeedUp(){
    return trackMotorSpeedUp;
  }
  public double getTrackMotorSpeedDown(){ return trackMotorSpeedDown;}
  public double getWheelMotorSpeed(){
    return wheelMotorSpeed;
  }
  public boolean isAmpZeroed(){
    return ampZeroed;
  }
  public boolean getAmpLimit(){return !ampLimit.get();}
  public int getMaxEncoderLimit(){ return maxEncoderLimit;}
  public int getMinEncoderLimit(){ return minEncoderLimit;}
  public double getHighSetpoint(){return highSetpoint;}
  public double getMidSetpoint(){return midSetpoint;}
  public double getLowSetpoint(){return lowSetpoint;}
  public double getP(){return P;}
  public double getI(){return I;}
  public double getD(){return D;}
  public double getAmpLocation(){return -trackMotor.getSelectedSensorPosition();}
  public boolean isAtSetpoint(){return ampPIDController.atSetpoint();}

  //setters
  public void setAmpZeroed(boolean newAmpZeroed){
    ampZeroed = newAmpZeroed;
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

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Amp");
      builder.addDoubleProperty("Amp Location", this::getAmpLocation, null);
      builder.addBooleanProperty("Amp Zeroed", this::isAmpZeroed, this::setAmpZeroed);
      builder.addBooleanProperty("At Setpoint", this::isAtSetpoint, null);
      builder.addBooleanProperty("Limit Switch", this::getAmpLimit, null);
      builder.addDoubleProperty("P", this::getP, this::setP);
      builder.addDoubleProperty("I", this::getI, this::setI);
      builder.addDoubleProperty("D", this::getD, this::setD);
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
