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

    private double currentMax = 0.0;

    private double timerAmount = 0.1;

    public double getCurrentMax() {
        return currentMax;
    }

    public void setCurrentMax(double value) {
        currentMax = value;
    }

    public double getTimerAmount() {
        return timerAmount;
    }

    public void setTimerAmount(double value) {
        timerAmount = value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("currentMax", this::getCurrentMax, this::setCurrentMax);
        builder.addDoubleProperty("timerAmount", this::getTimerAmount, this::setTimerAmount);
    }

    // Class for Right Climber Arm
    public static class ClimberRight extends SubsystemBase {

        private final DigitalInput limitSwitch;

        private final WPI_TalonSRX motor;

        public ClimberRight() {
            motor = new WPI_TalonSRX(Constants.Climber.kRMotor);
            motor.configFactoryDefault();
            motor.configVoltageCompSaturation(3);
            motor.setInverted(false);

            limitSwitch = new DigitalInput(Constants.Climber.kRLimitSwitch);
        }

        // returns the status of the right arm's limitswitch
        public boolean brokeLimit() {
            return !limitSwitch.get();
        }

        // inverts the motor direction
        public void invert() {
            motor.setInverted(!motor.getInverted());
        }

        // returns the amount of current the motor is using
        public double getMotorCurrent() {
            return motor.getMotorOutputVoltage();
        }

        // sets speed of right arm
        public void setSpeed(double speed) {
            motor.set(ControlMode.PercentOutput, speed);
        }

        // sets right arm speed to zero
        public void stop() {
            motor.set(ControlMode.PercentOutput, 0);
        }

        @Override
        public void periodic() {}

        @Override
        public void simulationPeriodic() {}

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("ClimberBrokeRight", this::brokeLimit, null);
        }
    }

    // Class for Left Climber Arm
    public static class ClimberLeft extends SubsystemBase {

        private final DigitalInput limitSwitch;

        private final WPI_TalonSRX motor;

        public ClimberLeft() {
            motor = new WPI_TalonSRX(Constants.Climber.kLMotor);
            motor.configFactoryDefault();
            motor.configVoltageCompSaturation(3);
            motor.setInverted(false);

            limitSwitch = new DigitalInput(Constants.Climber.kLLimitSwitch);
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
            return motor.getMotorOutputVoltage();
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
        public void periodic() {}

        @Override
        public void simulationPeriodic() {}

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("ClimberBrokeLeft", this::brokeLimit, null);
        }
    }
}