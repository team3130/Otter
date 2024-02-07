// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX leftFlywheel; // we should probably change these names once we learn more
    private final TalonFX rightFlywheel; // we should probably change these names once we learn more
    double proportionVolt = 1d;
    final VoltageOut leftVoltReq = new VoltageOut(0);
    final VoltageOut rightVoltReq = new VoltageOut(0);
    final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0).withSlot(0); // class instance
    final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0).withSlot(0);
    final double leftVelocitySetpoint = 8;
    private double flywheelVolts = 3;

    Slot0Configs slot0Configs; // gains for specific slot
    /*
      alternative way
      / / create a velocity closed-loop request, voltage output, slot 0 configs
      final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    */

    private double kS = 0.5;
    private double kV = 0.1;
    private double kP = 0.15;
    private double kI = 0;
    private double kD = 0;
    private double feedForwardVolt;

    private final WPI_TalonSRX indexMotor; // we should probably change these names once we learn more
    //private final WPI_TalonSRX motor6; // we should probably change these names once we learn more
    private double indexSpeed = 0.50;


    public Shooter() {
        leftFlywheel = new TalonFX(9);
        rightFlywheel = new TalonFX(8);

        leftFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        rightFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default

        leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheel.setNeutralMode(NeutralModeValue.Coast);

        rightFlywheel.setInverted(true);
        leftFlywheel.setInverted(true);

        slot0Configs = new Slot0Configs(); // gains for specific slot

        slot0Configs.kS = kS; // Add 0.05 V output to overcome static friction

        slot0Configs.kV = kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = kD; // output per unit of error derivative in velocity (output/ (rps/s))
        // leftFlywheel9.getConfigurator().apply(new Slot0Configs());

        indexMotor = new WPI_TalonSRX(10);

        indexMotor.configVoltageCompSaturation(4);

        indexMotor.configFactoryDefault();
        indexMotor.setNeutralMode(NeutralMode.Brake);
        indexMotor.setInverted(true);

    }

    public void runShooters() {
        leftFlywheel.setControl(leftVoltReq.withOutput(flywheelVolts));
        rightFlywheel.setControl(rightVoltReq.withOutput(flywheelVolts));
    }

    public void stopShooters() {
        leftFlywheel.setControl(leftVoltReq.withOutput(0));
        rightFlywheel.setControl(rightVoltReq.withOutput(0));
    }

    public void runIndexers() {
        indexMotor.set(ControlMode.PercentOutput, indexSpeed);
    }

    public double getIndexSpeed() {return indexSpeed;}
    public void setIndexSpeed(double speedy){
        indexSpeed = speedy;
    }

    public void stopIndexers() {
        indexMotor.set(ControlMode.PercentOutput, 0);
    }

    public void updateVelocityPID() {
        leftFlywheel.getConfigurator().apply(slot0Configs);
        rightFlywheel.getConfigurator().apply(slot0Configs);
        // leftFlywheel9.getConfigurator().apply(new Slot0Configs());
    }

    public void setFlywheelVelocity() {
        // velocityRequest.Slot = 0; // this is probably redudant now
        rightFlywheel.setControl(leftVelocityRequest.withVelocity(leftVelocitySetpoint).withFeedForward(feedForwardVolt));

        // ALT way: set velocity to 8 rps, add 0.5 V to overcome gravity
        // m_talonFX.setControl(velocityRequest.withVelocity(8).withFeedForward(0.5));
    }

    public void configureVelocitySlot() {
        slot0Configs.kS = kS; // Add 0.05 V output to overcome static friction

        slot0Configs.kV = kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = kD; // output per unit of error derivative in velocity (output/ (rps/s))
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
        return rightFlywheel.getSupplyVoltage().getValue();
    }
    public double getSpeed9() {
        return leftFlywheel.getSupplyVoltage().getValue();
    }

    public double getVelocityMotor8() {
        return rightFlywheel.getVelocity().getValue() * 60;
    }

    public double getVelocityMotor9() {
        return leftFlywheel.getVelocity().getValue() * 60;
    }

    public double getRightFlyVoltSupply() {
        return rightFlywheel.getSupplyVoltage().getValue();
    }

    public double getLeftFlywheelVoltSupply() {
        return leftFlywheel.getSupplyVoltage().getValue();
    }

    public double getRightFlyCurrent() {
        return rightFlywheel.getSupplyCurrent().getValue();
    }

    public double getFlywheelVolts(){
        return flywheelVolts;
    }
    public void setFlywheelVolts(double volt){flywheelVolts = volt;
    }
    public double getkS() { return kS; }
    public double getkV() { return kV; }
    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public void setkS(double newS) { slot0Configs.kS = newS; }
    public void setkV(double newV) { slot0Configs.kV = newV; }
    public void setkP(double newP) { slot0Configs.kP = newP; }
    public void setkI(double newI) { slot0Configs.kI = newI; }
    public void setkD(double newD) { slot0Configs.kD = newD; }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("speed 8", this::getSpeed8, null);
        builder.addDoubleProperty("speed 9", this::getSpeed9, null);
        builder.addDoubleProperty("proportion speed", this::getProportionVolt, this::setProportionVolt);

        builder.addDoubleProperty("8 real velocity", this::getVelocityMotor8, null);
        builder.addDoubleProperty("9 real velocity", this::getVelocityMotor9, null);

        builder.addDoubleProperty("right volt supply", this::getRightFlyVoltSupply, null);
        builder.addDoubleProperty("left volt supply", this::getLeftFlywheelVoltSupply, null);
        builder.addDoubleProperty("shooter volts", this::getFlywheelVolts, this::setFlywheelVolts);


        builder.addDoubleProperty("velocity kS", this::getkS, this::setkS);
        builder.addDoubleProperty("velocity kV", this::getkV, this::setkV);
        builder.addDoubleProperty("velocity kP", this::getkP, this::setkP);
        builder.addDoubleProperty("velocity kI", this::getkI, this::setkI);
        builder.addDoubleProperty("velocity kD", this::getkD, this::setkD);

        builder.setSmartDashboardType("Indexer");
        builder.addDoubleProperty("speed", this::getIndexSpeed, this::setIndexSpeed);
    }

    @Override
    public void simulationPeriodic() {

    }
}
