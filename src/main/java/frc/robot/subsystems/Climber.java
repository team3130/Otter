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
    private double currentMaxRight = 0.0;
    private double currentMaxLeft = 0.0;
    private double timerAmount = 0.1;
    private final DigitalInput limitSwitch;
    private final WPI_TalonSRX climberMotor;
    private boolean isReset;

    public Climber(int CANID, int limitSwitch) {
        climberMotor = new WPI_TalonSRX(CANID);
        climberMotor.configFactoryDefault();
        climberMotor.configVoltageCompSaturation(3);
        climberMotor.setInverted(false);

        this.limitSwitch = new DigitalInput(limitSwitch);
    }

    public double getCurrentMaxRight() {
        return currentMaxRight;
    }

    public void setCurrentMaxRight(double skibidi) {
        currentMaxRight = skibidi;
    }

    public double getCurrentMaxLeft() {
        return currentMaxLeft;
    }

    public void setCurrentMaxLeft(double skibidi) {
        currentMaxLeft = skibidi;
    }

    public double getTimerAmount() {
        return timerAmount;
    }

    public void setTimerAmount(double skibidi) {
        timerAmount = skibidi;
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
    public void setSpeed(double speed) {
        climberMotor.set(ControlMode.PercentOutput, speed);
    }

    // sets left arm speed to zero
    public void stop() {
        setSpeed(0);
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
        builder.addDoubleProperty("currentMaxRight", this::getCurrentMaxRight, this::setCurrentMaxRight);
        builder.addDoubleProperty("currentMaxLeft", this::getCurrentMaxLeft, this::setCurrentMaxLeft);
        builder.addDoubleProperty("timerAmount", this::getTimerAmount, this::setTimerAmount);
        builder.addBooleanProperty("ClimberIsReset", this::getIsReset, null);
    }

    public boolean getIsReset() {
        return isReset;
    }

    public void setIsReset(boolean reset) {
        isReset = reset;
    }
}