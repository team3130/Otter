// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewAmp extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final WPI_TalonSRX trackMotor; //motor for amp going up and down
  private final WPI_TalonSRX wheelMotor; //motor for amp wheels to turn (wheels at end of amp where note is)
  private final DigitalInput ampLimit; //limit switch on bottom of amp track

  //speeds
  private final double trackMotorSpeed = 0.1;
  private final double wheelMotorSpeed = 0.1;
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
  }

  public void ampIntake(){
    wheelMotor.set(ControlMode.PercentOutput, wheelMotorSpeed);
  }
  public void ampOuttake(){
    wheelMotor.set(ControlMode.PercentOutput, -wheelMotorSpeed);
  }
  public void wheelMotorStop(){
    wheelMotor.set(ControlMode.PercentOutput, 0);
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
