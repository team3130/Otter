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
    private double pitCheckingSpeed = 0.1;
    private final DigitalInput limitSwitch;
    private final WPI_TalonSRX climberMotor;
    private int joystickUsed;
    private boolean isClimberReset;

    private boolean invalidInput = false;

    public Climber(int CANID, int limitSwitchPort, int joystick, boolean inverted) {
        climberMotor = new WPI_TalonSRX(CANID);
        this.limitSwitch = new DigitalInput(limitSwitchPort);
        climberMotor.configFactoryDefault();
        climberMotor.configVoltageCompSaturation(3);
        climberMotor.setInverted(inverted);
        joystickUsed = joystick;

        isClimberReset = true;
    }

    // returns the status of the left arm's limitswitch
    public boolean brokeLimit() {
        return !limitSwitch.get();
    }
    public void setIsClimberReset(boolean bruh) { isClimberReset = bruh; }
    public boolean getIsClimberReset() { return this.isClimberReset; }
    public boolean getInvalidInput() {
        return invalidInput;
    }
    public void setInvalidInput(boolean valid){
        invalidInput = valid;
    }


    // sets speed of right arm
    public void setMotorCheckingSpeed() {
        climberMotor.set(ControlMode.PercentOutput, pitCheckingSpeed);
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(ControlMode.PercentOutput, speed);
    }

    // sets left arm speed to zero
    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }

    // returns the amount of current the motor is using
    public double getMotorCurrent() {
        return climberMotor.getSupplyCurrent();
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
    public double getPitCheckingSpeed(){
        return pitCheckingSpeed;
    }
    public void setPitCheckingSpeed(double speed){
        pitCheckingSpeed = speed;
    }
    public void setTimerAmount(double timer) {
        timerAmount = timer;
    }

    @Override
    public void periodic() {
        if (brokeLimit()){
            //TODO LEDS GREEN
        }
        if (getInvalidInput()){
            //TODO LEDS RED
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ClimberBrokeLeft", this::brokeLimit, null);
        builder.addDoubleProperty("currentMaxRight", this::getCurrentMax, this::setCurrentMax);
        builder.addDoubleProperty("timerAmount", this::getTimerAmount, this::setTimerAmount);
        builder.addDoubleProperty("checking speed", this::getPitCheckingSpeed, this::setPitCheckingSpeed);
        builder.addBooleanProperty("Climber is reset", this::getIsClimberReset, null);
    }
}