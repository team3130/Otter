// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final Solenoid intakePNMOne;
    private final DigitalInput intakeLimitSwitch;
    public Intake() {
        intakePNMOne = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.intakePNMChannel);
        intakeLimitSwitch = new DigitalInput(Constants.IDs.intakeLimitDIO);
    }

    public boolean getIntakeLimitSwitch() {
        return !intakeLimitSwitch.get();
    }

    public void toggleIntake() {
        intakePNMOne.toggle();
    }

    public void intakeUp() {
        intakePNMOne.set(false);
    }
    public void intakeDown(){
        intakePNMOne.set(true);
    }
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Intake");
            builder.addBooleanProperty("intake limit switch", this::getIntakeLimitSwitch, null);
        }
    }
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}