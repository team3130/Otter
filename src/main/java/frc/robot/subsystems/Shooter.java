// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX leftFlywheel9; // we should probably change these names once we learn more
    private final TalonFX rightFlywheel8; // we should probably change these names once we learn more
    double proportionVolt = 1.05;
    final double leftFlywheelVolt = 9;
    final double rightFlywheelVolt = 9 * proportionVolt ;
    final VoltageOut leftFlywheelVoltReq = new VoltageOut(0);
    final VoltageOut rightFlywheelVoltReq = new VoltageOut(0);

    public Shooter() {
        leftFlywheel9 = new TalonFX(9);
        rightFlywheel8 = new TalonFX(8);

        leftFlywheel9.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        rightFlywheel8.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        leftFlywheel9.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheel8.setNeutralMode(NeutralModeValue.Coast);

        rightFlywheel8.setInverted(true);
    }

    public void runShooters() {
        leftFlywheel9.setControl(leftFlywheelVoltReq.withOutput(9));
        rightFlywheel8.setControl(rightFlywheelVoltReq.withOutput(9));
    }

    public void stopShooters() {
        leftFlywheel9.setControl(leftFlywheelVoltReq.withOutput(0));
        rightFlywheel8.setControl(rightFlywheelVoltReq.withOutput(0));
    }

    @Override
    public void periodic() {
    }

    public double getProportionVolt() {
        return proportionVolt;
    }

    public void setProportionVolt(double newProp) {
        proportionVolt = newProp;
    }

    public double getSpeed8() {
        return rightFlywheel8.getSupplyVoltage().getValue();
    }
    public double getSpeed9() {
        return leftFlywheel9.getSupplyVoltage().getValue();
    }

    public double getVelocityMotor8() {
        return rightFlywheel8.getVelocity().getValue();
    }

    public double getVelocityMotor9() {
        return leftFlywheel9.getVelocity().getValue();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("speed 8", this::getSpeed8, null);
        builder.addDoubleProperty("speed 9", this::getSpeed9, null);
        builder.addDoubleProperty("proportion speed", this::getProportionVolt, this::setProportionVolt);
        builder.addDoubleProperty("8 real velocity", this::getVelocityMotor8, null);
        builder.addDoubleProperty("9 real velocity", this::getVelocityMotor9, null);

    }

    @Override
    public void simulationPeriodic() {

    }
}
