// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final WPI_TalonSRX motor7; // we should probably change these names once we learn more
    private final WPI_TalonSRX motor6; // we should probably change these names once we learn more
    private double speed = 0.80;

    private final DigitalInput indexerLimitSwitch1;

    public Indexer() {
        motor7 = new WPI_TalonSRX(7);
        motor6 = new WPI_TalonSRX(10);

        motor6.configVoltageCompSaturation(3);
        motor7.configVoltageCompSaturation(3);

        motor7.configFactoryDefault();
        motor6.configFactoryDefault();
        motor6.setInverted(true);

        indexerLimitSwitch1 = new DigitalInput(Constants.CAN.indexerLimitSwitch1);
    }

    public void runIndexers() {
        motor7.set(ControlMode.PercentOutput, speed);
        motor6.set(ControlMode.PercentOutput, speed);
    }

    public double getSpeed() {
        return speed;
    }

    public void stopIndexers() {
        motor6.set(ControlMode.PercentOutput, 0);
        motor7.set(ControlMode.PercentOutput, 0);
    }

    public boolean getIndexerLimitSwitch1(){
        return indexerLimitSwitch1.get();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Indexer");
        builder.addDoubleProperty("speed", this::getSpeed, null);
    }

    @Override
    public void simulationPeriodic() {  }
}