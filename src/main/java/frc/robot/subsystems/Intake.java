// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeIndexer;
  private double intakeSpeed = 0.50;

  public Intake() {
    intakeIndexer = new TalonSRX(0);
    intakeIndexer.configVoltageCompSaturation(4);

    intakeIndexer.configFactoryDefault();
    intakeIndexer.setNeutralMode(NeutralMode.Brake);
    intakeIndexer.setInverted(true);
  }

  public void runIntakeIndexer() {
    intakeIndexer.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void stopIntakeIndex() {
    intakeIndexer.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
