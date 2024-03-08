// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePNM extends SubsystemBase {
    private final Solenoid intakePNMOne;
    public IntakePNM() {
        intakePNMOne = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.intakePNMChannel);
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

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}