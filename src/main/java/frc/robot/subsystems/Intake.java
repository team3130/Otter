// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intakeMotor;
    private final Solenoid intakePNMOne;
    private final Solenoid intakePNMTwo;
    private final DigitalInput intakeLimitSwitch;
    private final DigitalInput breakbeam;
    private double dropTime = 0.2;
    public static double intakeNoteSetpoint = 300; // number of rotations from limit switch to when note should stop in intake

    private double outakeSpeed = -0.85;
    private double spintakeSpeed = 0.85;
    private double groundSpeed = 0.8;

    private double slowSpeed = .4; //S peed slower than dumbSpeed, in order to slow down the disk

    private boolean intakeHasNote;
    private boolean noteReadyToShoot;


    public Intake() {
        intakeMotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
        intakePNMOne = new Solenoid(Constants.CAN.intakesolenoid1, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);
        intakePNMTwo = new Solenoid(Constants.CAN.intakesolenoid2, PneumaticsModuleType.CTREPCM, PNM_INTAKE_ACTUATOR);

        intakeLimitSwitch = new DigitalInput(Constants.CAN.intakeLimitSwitch1);
        breakbeam = new DigitalInput(Constants.CAN.shooterBreakBeam);

        intakeMotor.configFactoryDefault();
        intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        intakeMotor.configVoltageCompSaturation(Constants.Intake.kMaxVoltageIntake);
        intakeMotor.enableVoltageCompensation(true);

        intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        intakeMotor.setInverted(false);

        intakeHasNote = false;
        noteReadyToShoot = false;
    }

    public void spintake() {
        intakeMotor.set(spintakeSpeed);
    }

    public void outtake(){
        intakeMotor.set(outakeSpeed);
    }

    public void groundIntake(){
        intakeMotor.set(groundSpeed);
    }

    public void slowTake(){
        intakeMotor.set(slowSpeed);
    }

    public void stoptake(){
        intakeMotor.set(0);
    }

    public void resetEncoders() {
        intakeMotor.setSelectedSensorPosition(0);
    }

    public boolean getIntakeLimitSwitch() {
        return intakeLimitSwitch.get();
    }

    public boolean getShooterBreakBeam(){
        return breakbeam.get();
    }

    public double getEncoderPosition() {
        return intakeMotor.getSelectedSensorPosition();
    }

    public void SolenoidToggle() {
        intakePNMOne.toggle();
        intakePNMTwo.toggle();
    }

    public void intakeDown(){
        intakePNMOne.set(true);
        intakePNMTwo.set(true);
    }
    public void intakeUp() {
        intakePNMOne.set(false);
        intakePNMTwo.set(false);
    }

    public boolean getIntakeHasNote() {
        return intakeHasNote;
    }
    public void setIntakeHasNote(boolean setNote) {
        intakeHasNote = setNote;
    }
    public boolean getNoteReadyToShoot() { return noteReadyToShoot; }
    public void setNoteReadyToShoot(boolean noteHand) { noteReadyToShoot = noteHand; }

    public double getDropTime() { return dropTime; }
    public void setDropTime(double dropTime) { this.dropTime = dropTime; }
    public double getIntakeNoteSetpoint() {return intakeNoteSetpoint;}
    public void setIntakeNoteSetpoint(double max) { intakeNoteSetpoint = max;}
    public double getGroundSpeed() { return groundSpeed; }
    public void setGroundSpeed(double newS) { groundSpeed = newS; }
    public void setSpintakeSpeed(double newSpeed) { spintakeSpeed = newSpeed; }
    public double getSpintakeSpeed() { return spintakeSpeed; }
    public void setOutakeSpeed(double newS) { outakeSpeed = newS; }
    public double getOutakeSpeed() { return outakeSpeed; }
    public double getSlowSpeed() { return slowSpeed; }
    public void setSlowSpeed(double slow) { slowSpeed = slow; }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addDoubleProperty("Intake Note Setpoint", this::getIntakeNoteSetpoint, this::setIntakeNoteSetpoint);

        builder.addDoubleProperty("Ground intake speed", this::getGroundSpeed, this::setGroundSpeed );
        builder.addDoubleProperty("Drop time", this::getDropTime, this::setDropTime);
        builder.addDoubleProperty("Dumb spintake speed", this::getSpintakeSpeed, this::setSpintakeSpeed);
        builder.addDoubleProperty("Dumb outtake speed", this::getOutakeSpeed, this::setOutakeSpeed);
        builder.addDoubleProperty("slow speed", this::getSlowSpeed, this::setSlowSpeed);
    }
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}