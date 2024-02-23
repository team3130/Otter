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

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intakeMotor;
    private final Solenoid intakePNMOne;
    private final DigitalInput intakeLimitSwitch;
    private double dropTime = 0.2;
    private double outakeSpeed = -1;
    private double spintakeSpeed = 1;

    private boolean trigger;


    public Intake() {
        intakeMotor = new WPI_TalonSRX(Constants.CAN.intakeIndexer);
        intakePNMOne = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.intakePNMChannel);

        intakeLimitSwitch = new DigitalInput(Constants.IDs.intakeLimitSwitch);

        intakeMotor.configFactoryDefault();
        intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        intakeMotor.configVoltageCompSaturation(9);
        intakeMotor.enableVoltageCompensation(true);

        intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        intakeMotor.setInverted(false);

        trigger = false;
    }

    public void setTestTrigger(boolean trig) {
        this.trigger = trig;
    }

    public boolean getTestTrigger(){ return this.trigger; }

    public void spintake() {
        intakeMotor.set(spintakeSpeed);
    }

    public void outtake(){
        intakeMotor.set(outakeSpeed);
    }

    public void stoptake(){
        intakeMotor.set(0);
    }

    public boolean getIntakeLimitSwitch() {
        return !intakeLimitSwitch.get();
    }

    public void toggleIntake() {
        intakePNMOne.toggle();
    }

    public void toggleIntakeIn() {
        intakePNMOne.set(true);
    }

    public double getDropTime() { return dropTime; }
    public void setDropTime(double dropTime) { this.dropTime = dropTime; }
    public void setSpintakeSpeed(double newSpeed) { spintakeSpeed = newSpeed; }
    public double getSpintakeSpeed() { return spintakeSpeed; }
    public void setOutakeSpeed(double newS) { outakeSpeed = newS; }
    public double getOutakeSpeed() { return outakeSpeed; }

    public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Intake");
            builder.addBooleanProperty("intake limit switch", this::getIntakeLimitSwitch, null);

            builder.addDoubleProperty("Drop time", this::getDropTime, this::setDropTime);
            builder.addDoubleProperty("Dumb spintake speed", this::getSpintakeSpeed, this::setSpintakeSpeed);
            builder.addDoubleProperty("Dumb outtake speed", this::getOutakeSpeed, this::setOutakeSpeed);

            builder.addBooleanProperty("test trigger", this::getTestTrigger, this::setTestTrigger);

    }
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}