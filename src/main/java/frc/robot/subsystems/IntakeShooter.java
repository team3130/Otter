// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeShooter extends SubsystemBase {

    private final DigitalInput intakeLimitSwitch;

    private final TalonFX topFlywheel; // we should probably change these names once we learn more
    private final TalonFX bottomFlywheel; // we should probably change these names once we learn more
    private double flywheelVolts = 5;

    final VoltageOut topVoltReq = new VoltageOut(0);
    final VoltageOut bottomVoltReq = new VoltageOut(0);
    final VelocityVoltage topVelocityRequest = new VelocityVoltage(0).withSlot(0);
    final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0).withSlot(1); // class instance

    //private CurrentLimitsConfigs topCurrentConfigs = new CurrentLimitsConfigs();
    //private CurrentLimitsConfigs bottomCurrentConfigs = new CurrentLimitsConfigs();


    double topVelocitySetpoint = 40;
    double bottomVelocitySetpoint = 40;



    private double topFeedForwardVolt = 0;
    private double bottomFeedForwardVolt = 0;
    Slot0Configs slot0Configs; // gains for top flywheel slot
    private double slot0_kS = 0; // DONT USE KS
    private double slot0_kV = 0.115; // OLD VALUE: 0.135;
    private double slot0_kP = 0.6; // OLD VALUE: 0.3;
    private double slot0_kI = 0; // OLD VALUE: 0
    private double slot0_kD = 0.2; // OLD VALUE: 0.01;

    Slot1Configs slot1Configs; // gains for bottom flywheel slot
    private double slot1_kS = 0; // DONT USE KS
    private double slot1_kV = 0.12; // OLD VALUE: 0.135;
    private double slot1_kP = 0.5; // OLD VALUE: 0.3;
    private double slot1_kI = 0; // OLD VALUE: 0
    private double slot1_kD = 0.2; // OLD VALUE: 0.01;


    ClosedLoopRampsConfigs topClosedLoopRamp;
    ClosedLoopRampsConfigs bottomClosedLoopRamp;


    private final WPI_TalonSRX indexer;
    private double indexerVoltageCompensation = 10;
    private double outakeSpeed = -1;
    private double spintakeSpeed = 1; // 10
    private double shooterSpindexSpeed = 1;
    private double autoSpintakeSpeed = 1; // 10
    private double autoShooterSpindexSpeed = 1;



    public IntakeShooter() {

        intakeLimitSwitch = new DigitalInput(Constants.IDs.intakeLimitDIO);


        topFlywheel = new TalonFX(Constants.CAN.shooterTopFlywheel);
        bottomFlywheel = new TalonFX(Constants.CAN.shooterBottomFlywheel);

        topFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        bottomFlywheel.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        
        topFlywheel.setNeutralMode(NeutralModeValue.Coast);
        bottomFlywheel.setNeutralMode(NeutralModeValue.Coast);

        topFlywheel.setInverted(true);
        bottomFlywheel.setInverted(true);

        //topFlywheel.getConfigurator().apply(topCurrentConfigs.withSupplyCurrentLimit(40));
        //bottomFlywheel.getConfigurator().apply(bottomCurrentConfigs.withSupplyCurrentLimit(40));

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

        slot1Configs.kS = slot1_kS; // Add 0.05 V output to overcome static friction
        slot1Configs.kV = slot1_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = slot1_kP; // 1/rps - An error of 1 rps results in 0.11 V output                        indexMotor = new WPI_TalonSRX(10);
        slot1Configs.kI = slot1_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot1Configs.kD = slot1_kD; // output per unit of error derivative in velocity (output/ (rps/s))         indexMotor.configVoltageCompSaturation(4);
        
        indexer = new WPI_TalonSRX(Constants.CAN.intakeIndexer);

        indexer.configFactoryDefault();
        indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        indexer.setNeutralMode(NeutralMode.Coast);
        indexer.configVoltageCompSaturation(indexerVoltageCompensation);
        indexer.enableVoltageCompensation(true);

        indexer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        //currLimitConfigs.currentLimit = 30;
        //indexer.configSupplyCurrentLimit(currLimitConfigs);

        indexer.setInverted(false);
        
        if (Constants.debugMode) {
            ShuffleboardTab tab = Shuffleboard.getTab("Shooter Velocity");
            tab.addDouble("Top Velocity Graph", this::getTopFlyVelocityRPS).withWidget("Graph").withPosition(0, 0).withSize(4, 3);
            tab.addDouble("Bottom Velocity Graph", this::getBottomFlyVelocityRPS).withWidget("Graph").withPosition(0, 3).withSize(4, 3);
            tab.addDouble("Top Flywheel Velocity", this::getTopFlyVelocityRPS).withPosition(4, 0).withSize(1, 1);
            tab.addDouble("Bottom Flywheel Velocity", this::getBottomFlyVelocityRPS).withPosition(4, 3).withSize(1, 1);
        }
    }
    

    public boolean getIntakeLimitSwitch() {
        return !intakeLimitSwitch.get();
    }

    public void rawRunShooterFlywheels() {
        topFlywheel.setControl(topVoltReq.withOutput(flywheelVolts));
        bottomFlywheel.setControl(bottomVoltReq.withOutput(flywheelVolts));
    }

    public void stopShooters() {
        topFlywheel.setControl(topVoltReq.withOutput(0));
        bottomFlywheel.setControl(bottomVoltReq.withOutput(0));
    }

    public void configureVelocitySlots() {
        topFlywheel.getConfigurator().apply(slot0Configs);
        bottomFlywheel.getConfigurator().apply(slot1Configs);
    }

    public void setFlywheelVelocity() {
        topFlywheel.setControl(topVelocityRequest.withVelocity(topVelocitySetpoint).withFeedForward(topFeedForwardVolt));
        bottomFlywheel.setControl(bottomVelocityRequest.withVelocity(bottomVelocitySetpoint).withFeedForward(bottomFeedForwardVolt));
    }

    public void autoSetFlywheelVelocity() {
        topFlywheel.setControl(topVelocityRequest.withVelocity(topVelocitySetpoint).withFeedForward(topFeedForwardVolt));
        bottomFlywheel.setControl(bottomVelocityRequest.withVelocity(bottomVelocitySetpoint).withFeedForward(bottomFeedForwardVolt));
    }

    public void updatePIDValues() {
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

    public double getTopFlyVelocityRPS() { return topFlywheel.getVelocity().getValue(); }
    public double getBottomFlyVelocityRPS() { return bottomFlywheel.getVelocity().getValue();}
    public double getTopFlyVelocityRPM() { return topFlywheel.getVelocity().getValue() * 60; }
    public double getBottomFlyVelocityRPM() { return bottomFlywheel.getVelocity().getValue() * 60;}

    public double getTopFlywheelSuppliedVolts() { return topFlywheel.getSupplyVoltage().getValue(); }
    public double getBottomFlywheelSuppliedVolts() { return bottomFlywheel.getSupplyVoltage().getValue(); }

    public double getTopCurrent() {return topFlywheel.getSupplyCurrent().getValue();}
    public double getBottomCurrent() {return bottomFlywheel.getSupplyCurrent().getValue();}

    public double getFlywheelRampTime() { return this.getFlywheelVolts();}
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


    public double getTopFlyVoltSupply() { return topFlywheel.getSupplyVoltage().getValue(); }
    public double getBottomFlyVoltSupply() { return bottomFlywheel.getSupplyVoltage().getValue(); }

    public double getTopFeedForwardVolt() { return topFeedForwardVolt; }
    public void setTopFeedForwardVolt(double lol) { topFeedForwardVolt = lol;}

    public double getBottomFeedForwardVolt() { return bottomFeedForwardVolt; }
    public void setBottomFeedForwardVolt(double lol) { bottomFeedForwardVolt = lol;}



    public void spintake() {
        indexer.set(spintakeSpeed);
    }

    public void outtake(){
        indexer.set(outakeSpeed);
    }

    public void shooterSpindex() {
        indexer.set(shooterSpindexSpeed);
    }

    public void stopIndexer() {
        indexer.set(0);
    }

    public void autoSpintake() {
        indexer.set(autoSpintakeSpeed);
    }

    public void autoShooterSpindex() {
        indexer.set(autoShooterSpindexSpeed);
    }

    public void setSpintakeSpeed(double newSpeed) { spintakeSpeed = newSpeed; }
    public double getSpintakeSpeed() { return spintakeSpeed; }
    public void setOutakeSpeed(double newS) { outakeSpeed = newS; }
    public double getOutakeSpeed() { return outakeSpeed; }
    public void setIndexerVoltageCompensation(double volts) { indexerVoltageCompensation = volts;}
    public double getIndexerVoltageCompensation() { return indexerVoltageCompensation; }
    public void setShooterSpindexSpeed(double lol) { shooterSpindexSpeed = lol;}
    public double getShooterSpindexSpeed() { return shooterSpindexSpeed; }

    public double getIndexerCurrent() { return indexer.getSupplyCurrent();}


@Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Intake");
            builder.addBooleanProperty("intake limit switch", this::getIntakeLimitSwitch, null);

            builder.setSmartDashboardType("Shooter");
            //builder.addDoubleProperty("proportion speed", this::getProportionVolt, this::setProportionVolt);
            builder.addDoubleProperty("top current", this::getTopCurrent, null);
            builder.addDoubleProperty("bottom current", this::getBottomCurrent, null);

            builder.addDoubleProperty("shooter volts", this::getFlywheelVolts, this::setFlywheelVolts);

            builder.addDoubleProperty("Top Flywheel Velocity (RPS)", this::getTopFlyVelocityRPS, null);
            builder.addDoubleProperty("Bottom Flywheel Velocity (RPS)", this::getBottomFlyVelocityRPS, null);

            builder.addDoubleProperty("Top Velocity Setpoint", this::getTopVelocitySetpoint, this::setTopVelocitySetpoint);
            builder.addDoubleProperty("Bottom Velocity Setpoint", this::getBottomVelocitySetpoint, this::setBottomVelocitySetpoint);

            builder.addDoubleProperty("Top RPM", this::getTopFlyVelocityRPM, null);
            builder.addDoubleProperty("Bottom RPM", this::getBottomFlyVelocityRPM, null);

            builder.addDoubleProperty("top volt supply", this::getTopFlyVoltSupply, null);
            builder.addDoubleProperty("bottom volt supply", this::getBottomFlyVoltSupply, null);

            builder.addDoubleProperty("top feedForward value", this::getTopFeedForwardVolt, this::setTopFeedForwardVolt);
            builder.addDoubleProperty("bottom feedforward value", this::getBottomFeedForwardVolt, this::setBottomFeedForwardVolt);

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
            builder.addDoubleProperty("Spintake speed", this::getSpintakeSpeed, this::setSpintakeSpeed);
            builder.addDoubleProperty("Outtake speed", this::getOutakeSpeed, this::setOutakeSpeed);
            builder.addDoubleProperty("Shooter Index speed", this::getShooterSpindexSpeed, this::setShooterSpindexSpeed);
            builder.addDoubleProperty("Indexer Voltage comp", this::getIndexerVoltageCompensation, this::setIndexerVoltageCompensation);
            builder.addDoubleProperty("indexer current", this::getIndexerCurrent, null);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
