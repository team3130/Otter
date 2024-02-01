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

public class Climber extends SubsystemBase {

    private double currentMaxRight = 0.0;

    private double currentMaxLeft = 0.0;

    private double timerAmount = 0.1;

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

    private final DigitalInput limitSwitch;

    private final WPI_TalonSRX motor;

    public Climber(int kMotor, int kLimitSwitch) {
        motor = new WPI_TalonSRX(kMotor);
        motor.configFactoryDefault();
        motor.configVoltageCompSaturation(3);
        motor.setInverted(false);

        limitSwitch = new DigitalInput(kLimitSwitch);
    }

    // returns the status of the left arm's limitswitch
    public boolean brokeLimit() {
        return !limitSwitch.get();
    }

    // inverts the motor direction
    public void invert() {
        motor.setInverted(!motor.getInverted());
    }

    // returns the amount of current the motor is using
    public double getMotorCurrent() {
        return motor.getSupplyCurrent();
    }

    // sets speed of right arm
    public void setSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    // sets left arm speed to zero
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
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
    }
}