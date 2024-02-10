// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Amp extends SubsystemBase {
  private final Solenoid pneumatic;
  private final DigitalInput ampLimit;
  private final WPI_TalonSRX ampMotor;
  private double intakeAmpSpeed = 0.1;
  private double outtakeAmpSpeed = -0.1;
  // the amount of seconds it takes for the amp to prime (pnematic to go up)

  public Amp() {
    pneumatic = new Solenoid(Constants.CAN.ampPCM, PneumaticsModuleType.CTREPCM, Constants.CAN.ampChannel);
    ampLimit = new DigitalInput(Constants.CAN.ampLimitSwitch);
    ampMotor = new WPI_TalonSRX(Constants.CAN.ampMotor);
    ampMotor.configFactoryDefault();
    ampMotor.configVoltageCompSaturation(3);
    ampMotor.setInverted(false);
  }

  // toggles the pneumatic to prop-up the amp arm
  public void toggleAmp() {
    pneumatic.toggle();
  }

  // toggles the pneumatic to tuck in the amp arm
  public void unPrimeAmp() {
    pneumatic.set(false);
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

  public boolean getAmpLimitSwitch() {
    return ampLimit.get();
  }
  public boolean getPneumaticState() {
    return pneumatic.get();
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
    builder.setSmartDashboardType("Amp");
    builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
    builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
    builder.addBooleanProperty("Limit Switch", this::getAmpLimitSwitch, null);
    builder.addBooleanProperty("Pneumatic Status", this::getPneumaticState, null);
  }

}
