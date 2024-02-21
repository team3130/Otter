// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Amp extends SubsystemBase {
  private final Solenoid ampPneumatic;
  private final DigitalInput ampLimit;
  private final WPI_TalonSRX ampMotor;
  private double intakeAmpSpeed = 1;
  private double outtakeAmpSpeed = -1;
  private Timer timer = new Timer();
  private final XboxController controller;
  // the amount of seconds it takes for the amp to prime (pnematic to go up)

  public Amp() {
    ampPneumatic = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.ampPNMChannel);
    ampLimit = new DigitalInput(Constants.IDs.ampLimitSwitch);
    ampMotor = new WPI_TalonSRX(Constants.CAN.ampMotor);
    ampMotor.configFactoryDefault();
    ampMotor.configVoltageCompSaturation(8);
    ampMotor.setInverted(false);
  }

  // toggles the pneumatic to prop-up the amp arm
  public void toggleAmp() {
    ampPneumatic.toggle();
  }

  // toggles the pneumatic to tuck in the amp arm
  public void unPrimeAmp() {
    ampPneumatic.set(false);
  }

  // spins the motor to intake notes into the amp
  public void intakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, intakeAmpSpeed);
  }

  // spins the motor to eject notes from the amp
  public void outtakeAmp() {
    ampMotor.set(ControlMode.PercentOutput, outtakeAmpSpeed);
  }

  // stops the motor to prevent the note from getting destroyed in the amp
  public void ampMotorStop() {
    ampMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getLimitSwitch() {
    return !ampLimit.get();
  }
  public boolean getPneumaticState() {
    return ampPneumatic.get();
  }

  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }
  public double getOuttakeAmpSpeed() { return outtakeAmpSpeed; }
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }
  public void setOuttakeAmpSpeed(double speed) {
    outtakeAmpSpeed = speed;
  }

  @Override
  public void periodic() {
    /*if(getLimitSwitch()){
      timer.start();
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
      if (timer.hasElapsed(0.8)){
        timer.reset();
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
      }
    }*/
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * exports data to Shuffleboard
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Amp");
      builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
      builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
      builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
      builder.addBooleanProperty("Pneumatic Status", this::getPneumaticState, null);
    }
  }

}
