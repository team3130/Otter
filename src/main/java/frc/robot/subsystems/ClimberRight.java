// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class ClimberRight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final DigitalInput limitSwitch;

  private final WPI_TalonSRX motor;

  private boolean ratchetDir = true;


  public ClimberRight() {
    motor = new WPI_TalonSRX(Constants.Climber.kRMotor);
    motor.configFactoryDefault();
    motor.setInverted(false);


    limitSwitch = new DigitalInput(Constants.Climber.kRLimitSwitch);

  }

  public boolean brokeRight() {
    return !limitSwitch.get();
  }

  public void setSpeedRight(double speed) {
    if (!ratchetDir) {
      speed *= -1;
    }
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void stopRight() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getRatchetDir() {
    return ratchetDir;
  }

  public void setRatchetDir(boolean value) {
    this.ratchetDir = value;
  }

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
      builder.addBooleanProperty("ClimberBrokeRight", this::brokeRight, null);
      builder.addBooleanProperty("RightRatchetDirection", this::getRatchetDir, this::setRatchetDir);
  }
}
