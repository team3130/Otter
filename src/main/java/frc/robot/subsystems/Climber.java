// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends SubsystemBase {
    private double currentMax = 0.0;
    private double timerAmount = 0.1;
    private double checkingSpeed = 0.1;
    private final DigitalInput limitSwitch;
    private final WPI_TalonSRX climberMotor;
    private int joystickUsed;

    public Climber(int CANID, int limitSwitchPort, int joystick) {
        climberMotor = new WPI_TalonSRX(CANID);
        climberMotor.configFactoryDefault();
        climberMotor.configVoltageCompSaturation(3);
        climberMotor.setInverted(false);
        joystickUsed = joystick;
        this.limitSwitch = new DigitalInput(limitSwitchPort);
    }

    public double getCurrentMax() {
        return currentMax;
    }

    public void setCurrentMax(double current) {
        currentMax = current;
    }

    public double getTimerAmount() {
        return timerAmount;
    }
    public int getJoystick() {
        return joystickUsed;
    }
    public double getCheckingSpeed(){
        return checkingSpeed;
    }
    public void setCheckingSpeed(double speed){
        checkingSpeed= speed;
    }

    public void setTimerAmount(double timer) {
        timerAmount = timer;
    }

    // returns the status of the left arm's limitswitch
    public boolean brokeLimit() {
        return !limitSwitch.get();
    }

    // inverts the motor direction
    public void invert() {
        climberMotor.setInverted(!climberMotor.getInverted());
    }

    // returns the amount of current the motor is using
    public double getMotorCurrent() {
        return climberMotor.getSupplyCurrent();
    }

    // sets speed of right arm
    public void setMotorCheckingSpeed() {
        climberMotor.set(ControlMode.PercentOutput, checkingSpeed);
    }
    public void setMotorSpeed(double speed) {
        climberMotor.set(ControlMode.PercentOutput, speed);
    }


    // sets left arm speed to zero
    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ClimberBrokeLeft", this::brokeLimit, null);
        builder.addDoubleProperty("currentMaxRight", this::getCurrentMax, this::setCurrentMax);
        builder.addDoubleProperty("timerAmount", this::getTimerAmount, this::setTimerAmount);
        builder.addDoubleProperty("checking speed", this::getCheckingSpeed, this::setCheckingSpeed);

    }
}