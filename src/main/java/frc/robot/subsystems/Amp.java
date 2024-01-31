// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {

  private final Solenoid pneumatic;
  private final DigitalInput ampLimit;
  private final TalonFX ampMotor;
  private double intakeAmpSpeed = 0.1;
  private double outtakeAmpSpeed = -0.1;

  /** Creates a new ExampleSubsystem. */
  public Amp() {
    pneumatic = new Solenoid(Constants.CAN.CAN_PCM, PneumaticsModuleType.CTREPCM, Constants.CAN.CAN_AmpChannel);
    ampLimit = new DigitalInput(Constants.CAN.CAN_AmpLimitSwitch);
    ampMotor = new TalonFX(Constants.CAN.CAN_AmpMotor);
  }

  /**
   * @return the status of the limit switch
   */
  public boolean getLimitSwitch() {
    return ampLimit.get();
  }

  /**
   * primes the amp arm
   */
  public void primeAmp() {
    pneumatic.set(true);
  }

  /**
   * tucks back the amp arm
   */
  public void unPrimeAmp() {
    pneumatic.set(false);
  }

  /**
   * spins the amp-take motor to intake notes
   */
  public void intakeAmp() {
    ampMotor.set(intakeAmpSpeed);
  }

  /**
   * spins the amp-take motor to outtake notes
   */
  public void outtakeAmp() {
    ampMotor.set(outtakeAmpSpeed);
  }

  /**
   * stops the amp-take motor
   */
  public void motorStop() {
    ampMotor.set(0);
  }

  /**
   * @return gets value of intake amp speed
   */
  public double getIntakeAmpSpeed() {
    return intakeAmpSpeed;
  }

  /**
   * @return gets value of outtake amp speed
   */
  public double getOuttakeAmpSpeed() {
    return outtakeAmpSpeed;
  }

  /**
   * @return sets value of intake amp speed
   */
  public void setIntakeAmpSpeed(double speed) {
    intakeAmpSpeed = speed;
  }

  /**
   * @return sets value of outtake amp speed
   */
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
   * exports data to sShuffleboard
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Amp");
    builder.addDoubleProperty("Intake Amp Speed", this::getIntakeAmpSpeed, this::setIntakeAmpSpeed);
    builder.addDoubleProperty("Outtake Amp Speed", this::getOuttakeAmpSpeed, this::setOuttakeAmpSpeed);
    builder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
  }

}
