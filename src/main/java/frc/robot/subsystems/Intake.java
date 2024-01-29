// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final VictorSPX intakeMotor; // we should probably change these names once we learn more
  private final double speed = 0.8;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    intakeMotor = new VictorSPX(5);
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.configVoltageCompSaturation(3);

    intakeMotor.setInverted(false);
  }

  public void runIntake() {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0d);
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
