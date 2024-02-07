// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PNM_INTAKE_ACTUATOR;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX intakeMotor;
  private final Solenoid intakesolenoid1;
  private final Solenoid intakesolenoid2;

  private final DigitalInput limitSwitch1;
  private double dropTime = 0.2;

  public static double maxIntakeTicks = 300; //This number represents the distance from the Limit Switch to the point where we want the disk to stop

  public static double bufferIntakeTicks = 200; //This number represents the distance that we want the disk to go before slowing down.

  public static double slowSpeed = .4; //Speed slower than dumbSpeed, in order to slow down the disk
  public static double groundSpeed = .8; //Speed slower than dumbSpeed, in order to slow down the disk

  public static double dumbSpeed = .85; //Speeed value for the high speed intake/outake


  public Intake() {
    intakeMotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    intakesolenoid1 = new Solenoid(Constants.CAN.intakesolenoid1, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    intakesolenoid2 = new Solenoid(Constants.CAN.intakesolenoid2, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    limitSwitch1 = new DigitalInput(Constants.CAN.intakeLimitSwitch1);

    intakeMotor.configFactoryDefault();
    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    intakeMotor.configVoltageCompSaturation(Constants.Intake.kMaxVoltageIntake);

    intakeMotor.enableVoltageCompensation(true);

    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    intakeMotor.setInverted(false);

  }
  /*
   * Example command factory method.
   *
   * @return a command
   */
  /*
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public double getPosition() {
    return intakeMotor.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return intakeMotor.getSelectedSensorVelocity();
  }
  public double getMaxIntakeTicks() {
    return maxIntakeTicks;
  }

  public void setMaxIntakeTicks(double MaxTicks) {
    maxIntakeTicks = MaxTicks;
  }

  public double getSlowSpeed() {
    return slowSpeed;
  }

  public void setSlowSpeed(double slowSpeed1) {
    slowSpeed = slowSpeed1;
  }

  public double getDumbSpeed() {
    return dumbSpeed;
  }

  public void setDumbSpeed(double dumbSpeed1) {
    dumbSpeed = dumbSpeed1;
  }
 public boolean intakeLimitSwitch1(){
    return limitSwitch1.get();
  }
  public void DumbIntake(){
    intakeMotor.set(dumbSpeed);
  }
  public void DumbOuttake(){
    intakeMotor.set(-dumbSpeed);
  }
    public void GroundIntake(){
        intakeMotor.set(groundSpeed);
    }

    public void gentleIntake(){
    intakeMotor.set(slowSpeed);
  }

  public void resetEncoders() {
    intakeMotor.setSelectedSensorPosition(0);
  }
  public boolean getLimitSwitch(){
      return limitSwitch1.get();
  }
  

  public void Stoptake(){
    intakeMotor.set(0);
  }


//This function is only ever called with the pneumatics being extending out of the frame perimeter. Need to consult with operator for preference for control scheme

  public void SolenoidToggle() {
    intakesolenoid1.toggle();
    intakesolenoid2.toggle();
  }
  public void intakeDown(){
      intakesolenoid1.set(true);
      intakesolenoid2.set(true);
  }
  public void intakeUp(){
        intakesolenoid1.set(false);
        intakesolenoid2.set(false);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Distance from Limit Switch to Stop point", this::getMaxIntakeTicks, this::setMaxIntakeTicks);
    builder.addDoubleProperty("slowSpeed", this::getSlowSpeed, this::setSlowSpeed);
    builder.addDoubleProperty("dumbSpeed", this::getDumbSpeed, this::setDumbSpeed);
    builder.addDoubleProperty("drop time", this::getDropTime, this::setDropTime);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    public double getDropTime() {
        return dropTime;
    }

    public void setDropTime(double dropTime) {
        this.dropTime = dropTime;
    }
}