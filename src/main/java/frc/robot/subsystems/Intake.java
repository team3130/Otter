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

import static frc.robot.Constants.Intake.dumbSpeed;
import static frc.robot.Constants.PNM_INTAKE_ACTUATOR;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX intakemotor;
  private final Solenoid intakesolenoid1;
  private final Solenoid intakesolenoid2;

  private final DigitalInput limitSwitch1;

  public static double maxIntakeTicks = 300; //This number represents the distance from the Limit Switch to the point where we want the disk to stop

  public static double bufferIntakeTicks = 200; //This number represents the distance that we want the disk to go before slowing down.


  public Intake() {
    intakemotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    intakesolenoid1 = new Solenoid(Constants.CAN.intakesolenoid1, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    intakesolenoid2 = new Solenoid(Constants.CAN.intakesolenoid2, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
    limitSwitch1 = new DigitalInput(Constants.CAN.intakeLimitSwitch1);

    intakemotor.configFactoryDefault();

    intakemotor.configVoltageCompSaturation(Constants.Intake.kMaxVoltageIntake);

    intakemotor.enableVoltageCompensation(true);

    intakemotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    intakemotor.setInverted(false);

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
    return intakemotor.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return intakemotor.getSelectedSensorVelocity();
  }
  public double getMaxIntakeTicks() {
    return maxIntakeTicks;
  }

  public void setMaxIntakeTicks(double MaxTicks) {
    maxIntakeTicks = MaxTicks;
  }

  public double getBufferIntakeTicks() {
    return bufferIntakeTicks;
  }

  public void setBufferIntakeTicks(double BufferTicks) {
    bufferIntakeTicks = BufferTicks;
  }



  //This function is only ever called with the pneumatics being extending out of the frame perimeter. Need to consult with operator for preference for control scheme
  public void SmartIntake() {
    if (getPosition() < bufferIntakeTicks) {
      // this checks to see how far the motor has traveled. Because decelerating over a large time period is slower than decelerating over a shorter one,
      // the intake will maintain its current speed for a short time until the ticks surpass the buffer tick count
      DumbIntake();
    } else {
        intakemotor.set(((dumbSpeed * -1) / (maxIntakeTicks - bufferIntakeTicks)) * (getPosition() - bufferIntakeTicks) + dumbSpeed);
      //dumb speed = ds, max ticks = mt, buffer ticks = bt, intakemotor.getSelected sensor position = x, output speed = y. All non x variables are constants which can be changed in tuning
      // y= ((-ds)/(mt-bt))(x-bt)+ds
      // this equation is based on the point slope equation with ((-ds)/(mt-bt)) being the equation for slope, and bt and ds being for x and y coordinates of where the two equations intersect
      //currently ds = .85, mt = 300, and bt = 200.
      // when paired with the above equation y = .85 in a piecewise function
      // x < 200 { y = .85
      // x >= 200 { y= ds*((mt-(x-bt))/mt)
    }
  }
  public boolean intakeLimitSwitch1(){
    return limitSwitch1.get();
  }
  public void DumbIntake(){
    intakemotor.set(dumbSpeed);
  }
  public void DumbOuttake(){
    intakemotor.set(-dumbSpeed);
  }

  public void ResetEncoders() {
    intakemotor.setSelectedSensorPosition(0);
  }
  

  public void Stoptake(){
    intakemotor.set(0);
  }

  public void limitSwitchCheck(){
    if(intakeLimitSwitch1()) {
      ResetEncoders();
    }
  }
//This function is only ever called with the pneumatics being extending out of the frame perimeter. Need to consult with operator for preference for control scheme

  public void SolenoidToggle() {
    intakesolenoid1.toggle();
    intakesolenoid2.toggle();
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Distance from Limit Switch to Stop point", this::getMaxIntakeTicks, this::setMaxIntakeTicks);
    builder.addDoubleProperty("Distance from limit Switch to Slowing Down point", this::getBufferIntakeTicks, this::setBufferIntakeTicks);
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