// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewAmp extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final WPI_TalonSRX trackMotor; //motor for amp going up and down
  private final WPI_TalonSRX wheelMotor; //motor for amp wheels to turn (wheels at end of amp where note is)
  private final DigitalInput ampLimit; //limit switch on bottom of amp track
  private PIDController ampPIDController; //pid controller for amp
  private double P = 0.0;
  private double I = 0.0;
  private double D = 0.0;

  //speeds
  private final double trackMotorSpeed = 0.1;
  private final double wheelMotorSpeed = 0.1;

  //setpoints
  private final int highSetpoint = 0;
  private final int midSetpoint = 0;
  private final int lowSetpoint = 0;

  private boolean ampZeroed = false;
  private final int maxEncoder = 20000;
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
    trackMotor.setInverted(false);
    wheelMotor.setInverted(false);

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
  public void trackMotorUp(){trackMotor.set(ControlMode.PercentOutput, trackMotorSpeed);}
  public void trackMotorDown(){trackMotor.set(ControlMode.PercentOutput, -trackMotorSpeed);}
  public void trackMotorStop() {trackMotor.set(ControlMode.PercentOutput, 0);}

  public void ampLimitReached(){
    if(getAmpLimit()){
      trackMotorStop();
      resetAmpEncoder();
    }
  }
  public void resetAmpController(){
    ampPIDController.setTolerance(100);
    ampPIDController.setPID(P,I,D);
  }
  public void runAmpController(double setpoint){
    ampPIDController.calculate(getAmpLocation(), setpoint);
  }
  public void resetAmpEncoder(){
    trackMotor.setSelectedSensorPosition(0);
  }

  //getters
  public double getTrackMotorSpeed(){
    return trackMotorSpeed;
  }
  public double getWheelMotorSpeed(){
    return wheelMotorSpeed;
  }
  public boolean isAmpZeroed(){
    return ampZeroed;
  }
  public boolean getAmpLimit(){return !ampLimit.get();}
  public double getHighSetpoint(){return highSetpoint;}
  public double getMidSetpoint(){return midSetpoint;}
  public double getLowSetpoint(){return lowSetpoint;}
  
  public double getP(){return P;}
  public double getI(){return I;}
  public double getD(){return D;}
  public double getAmpLocation(){return trackMotor.getSelectedSensorPosition();}
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






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
