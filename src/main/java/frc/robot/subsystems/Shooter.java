// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX topFlywheel; // we should probably change these names once we learn more
    private final TalonFX bottomFlywheel; // we should probably change these names once we learn more
    // double proportionVolt = 1.05;
    private final TalonSRX indexMotor;
    private double shooterIndexSpeed = 0.50;
    private double flywheelRampTime = 0;


    final VoltageOut topVoltReq = new VoltageOut(0);
    final VoltageOut bottomVoltReq = new VoltageOut(0);
    final VelocityVoltage topVelocityRequest = new VelocityVoltage(0).withSlot(1);
    final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0).withSlot(0); // class instance
    double topVelocitySetpoint = 8;
    double bottomVelocitySetpoint = 8;
    private double flywheelVolts = 3;

    Slot0Configs slot0Configs; // gains for top flywheel slot
    private double slot0_kS = 0; // DONT USE KS
    private double slot0_kV = 0; // OLD VALUE: 0.135;
    private double slot0_kP = 0; // OLD VALUE: 0.3;
    private double slot0_kI = 0; // OLD VALUE: 0
    private double slot0_kD = 0; // OLD VALUE: 0.01;
    Slot1Configs slot1Configs; // gains for bottom flywheel slot
    private double slot1_kS = 0; // DONT USE KS
    private double slot1_kV = 0; // OLD VALUE: 0.135;
    private double slot1_kP = 0; // OLD VALUE: 0.3;
    private double slot1_kI = 0; // OLD VALUE: 0
    private double slot1_kD = 0; // OLD VALUE: 0.01;

    private double topFeedForwardVolt;
    private double bottomFeedForwardVolt;
    ClosedLoopRampsConfigs topClosedLoopRamp;
    ClosedLoopRampsConfigs bottomClosedLoopRamp;


    public Shooter() {
        topFlywheel = new TalonFX(20);
        bottomFlywheel = new TalonFX(21);
        indexMotor = new TalonSRX(22);
        indexMotor.configFactoryDefault();
        indexMotor.setNeutralMode(NeutralMode.Brake);
        indexMotor.setInverted(true);

        topFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        bottomFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default

        topFlywheel.setNeutralMode(NeutralModeValue.Coast);
        bottomFlywheel.setNeutralMode(NeutralModeValue.Coast);

        topFlywheel.setInverted(true);
        bottomFlywheel.setInverted(true);

        // idk if these ramp rates do anything :(
        topFlywheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(1));
        bottomFlywheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(1));

        slot0Configs = new Slot0Configs(); // gains for top flywheel slot
        slot1Configs = new Slot1Configs(); // gains for bottom flywheel slot

        slot0Configs.kS = slot0_kS; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = slot0_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = slot0_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = slot0_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = slot0_kD; // output per unit of error derivative in velocity (output/ (rps/s))
        // leftFlywheel9.getConfigurator().apply(new Slot0Configs());

        slot1Configs.kS = slot1_kS; // Add 0.05 V output to overcome static friction
        slot1Configs.kV = slot1_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = slot1_kP; // 1/rps - An error of 1 rps results in 0.11 V output                        indexMotor = new WPI_TalonSRX(10);
        slot1Configs.kI = slot1_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot1Configs.kD = slot1_kD; // output per unit of error derivative in velocity (output/ (rps/s))         indexMotor.configVoltageCompSaturation(4);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter Velocity");
        tab.addDouble("Top Velocity Graph", this::getTopFlyVelocityRPS).withWidget("Graph").withPosition(0, 0).withSize(4, 3);
        tab.addDouble("Bottom Velocity Graph", this::getBottomFlyVelocityRPS).withWidget("Graph").withPosition(0, 3).withSize(4, 3);
        tab.addDouble("Top Flywheel Velocity", this::getTopFlyVelocityRPS).withPosition(4, 0).withSize(1, 1);
        tab.addDouble("Bottom Flywheel Velocity", this::getBottomFlyVelocityRPM).withPosition(4, 3).withSize(1, 1);
    }

    public void runShooters() {
        topFlywheel.setControl(topVoltReq.withOutput(flywheelVolts));
        bottomFlywheel.setControl(bottomVoltReq.withOutput(flywheelVolts));
    }

    public void stopShooters() {
        topFlywheel.setControl(topVoltReq.withOutput(0));
        bottomFlywheel.setControl(bottomVoltReq.withOutput(0));
    }

    public void runIndexers() {
        indexMotor.set(ControlMode.PercentOutput, shooterIndexSpeed);
    }

    public void stopIndexers() {
        indexMotor.set(ControlMode.PercentOutput, 0);
    }

    public void updateVelocityPID() {
        topFlywheel.getConfigurator().apply(slot0Configs);
        bottomFlywheel.getConfigurator().apply(slot1Configs);
    }

    public void setFlywheelVelocity() {
        topFlywheel.setControl(topVelocityRequest.withVelocity(topVelocitySetpoint).withFeedForward(topFeedForwardVolt));
        // bottomFlywheel.setControl(topVelocityRequest.withVelocity(topVelocitySetpoint).withFeedForward(topFeedForwardVolt));
    }

    public void configureVelocitySlots() {
        slot0Configs.kS = slot0_kS; // Add 0.05 V output to overcome static friction

        slot0Configs.kV = slot0_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = slot0_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = slot0_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = slot0_kD; // output per unit of error derivative in velocity (output/ (rps/s))

        slot1Configs.kS = slot1_kS; // Add 0.05 V output to overcome static friction

        slot1Configs.kV = slot1_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = slot1_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot1Configs.kI = slot1_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot1Configs.kD = slot1_kD; // output per unit of error derivative in velocity (output/ (rps/s))
    }

    @Override
    public void periodic() {
    }

    /*
    public double getProportionVolt() {
        return proportionVolt;
    }

    public void setProportionVolt(double newProp) {
        proportionVolt = newProp;
    }
     */

    public double getShooterIndexSpeed() {return shooterIndexSpeed;}
    public void setShooterIndexSpeed(double speedy){ shooterIndexSpeed = speedy;}

    public double getTopFlyVelocityRPS() { return topFlywheel.getVelocity().getValue(); }
    public double getBottomFlyVelocityRPS() { return bottomFlywheel.getVelocity().getValue();}
    public double getTopFlyVelocityRPM() { return topFlywheel.getVelocity().getValue() * 60; }
    public double getBottomFlyVelocityRPM() { return bottomFlywheel.getVelocity().getValue() * 60;}

    public double getTopFlyVoltSupply() { return topFlywheel.getSupplyVoltage().getValue(); }
    public double getBottomFlyVoltSupply() { return bottomFlywheel.getSupplyVoltage().getValue(); }

    public double getTopCurrent() {return topFlywheel.getSupplyCurrent().getValue();}
    public double getBottomCurrent() {return bottomFlywheel.getSupplyCurrent().getValue();}

    public double getFlywheelRampTime() { return this.getFlywheelVolts();}
    public void setFlywheelRampTime(double newTime) { this.flywheelRampTime = newTime;}


    public double getFlywheelVolts(){ return flywheelVolts;}
    public void setFlywheelVolts(double volt){flywheelVolts = volt;}

    public double getSlot0_kS() { return slot0_kS; }
    public double getSlot0_kV() { return slot0_kV; }
    public double getSlot0_kP() { return slot0_kP; }
    public double getSlot0_kI() { return slot0_kI; }
    public double getSlot0_kD() { return slot0_kD; }
    public void setSlot0_kS(double newS) { this.slot0_kS = newS; }
    public void setSlot0_kV(double newV) { this.slot0_kV = newV; }
    public void setSlot0_kP(double newP) { this.slot0_kP = newP; }
    public void setSlot0_kI(double newI) { this.slot0_kI = newI; }
    public void setSlot0_kD(double newD) { this.slot0_kD = newD; }

    public double getSlot1_kS() { return slot1_kS; }
    public double getSlot1_kV() { return slot1_kV; }
    public double getSlot1_kP() { return slot1_kP; }
    public double getSlot1_kI() { return slot1_kI; }
    public double getSlot1_kD() { return slot1_kD; }
    public void setSlot1_kS(double newS) { this.slot1_kS = newS; }
    public void setSlot1_kV(double newV) { this.slot1_kV = newV; }
    public void setSlot1_kP(double newP) { this.slot1_kP = newP; }
    public void setSlot1_kI(double newI) { this.slot1_kI = newI; }
    public void setSlot1_kD(double newD) { this.slot1_kD = newD; }

    public void setTopVelocitySetpoint(double newVelocity) {this.topVelocitySetpoint = newVelocity;}
    public void setBottomVelocitySetpoint(double newVelocity) {this.bottomVelocitySetpoint = newVelocity;}
    public double getTopVelocitySetpoint() {return this.topVelocitySetpoint;}
    public double getBottomVelocitySetpoint() {return this.bottomVelocitySetpoint;}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        //builder.addDoubleProperty("proportion speed", this::getProportionVolt, this::setProportionVolt);
        builder.addDoubleProperty("Flywheel Ramp Time", this::getFlywheelRampTime, this::setFlywheelRampTime);
        builder.addDoubleProperty("shooter volts", this::getFlywheelVolts, this::setFlywheelVolts);

        builder.addDoubleProperty("Top Flywheel Velocity (RPS)", this::getTopFlyVelocityRPS, null);
        builder.addDoubleProperty("Bottom Flywheel Velocity (RPS)", this::getBottomFlyVelocityRPS, null);

        builder.addDoubleProperty("Top Velocity Setpoint", this::getTopVelocitySetpoint, this::setTopVelocitySetpoint);
        builder.addDoubleProperty("Bottom Velocity Setpoint", this::getBottomVelocitySetpoint, this::setBottomVelocitySetpoint);

        builder.addDoubleProperty("Top RPM", this::getTopFlyVelocityRPM, null);
        builder.addDoubleProperty("Bottom RPM", this::getBottomFlyVelocityRPM, null);

        builder.addDoubleProperty("top volt supply", this::getTopFlyVoltSupply, null);
        builder.addDoubleProperty("bottom volt supply", this::getBottomFlyVoltSupply, null);

        builder.addDoubleProperty("slot 0 kS", this::getSlot0_kS, this::setSlot0_kS);
        builder.addDoubleProperty("slot 0 kV", this::getSlot0_kV, this::setSlot0_kV);
        builder.addDoubleProperty("slot 0 kP", this::getSlot0_kP, this::setSlot0_kP);
        builder.addDoubleProperty("slot 0 kI", this::getSlot0_kI, this::setSlot0_kI);
        builder.addDoubleProperty("slot 0 kD", this::getSlot0_kD, this::setSlot0_kD);

        builder.addDoubleProperty("slot 1 kS", this::getSlot1_kS, this::setSlot1_kS);
        builder.addDoubleProperty("slot 1 kV", this::getSlot1_kV, this::setSlot1_kV);
        builder.addDoubleProperty("slot 1 kP", this::getSlot1_kP, this::setSlot1_kP);
        builder.addDoubleProperty("slot 1 kI", this::getSlot1_kI, this::setSlot1_kI);
        builder.addDoubleProperty("slot 1 kD", this::getSlot1_kD, this::setSlot1_kD);

        builder.setSmartDashboardType("Indexer");
        builder.addDoubleProperty("speed", this::getShooterIndexSpeed, this::setShooterIndexSpeed);
    }

    @Override
    public void simulationPeriodic() {

    }
}
