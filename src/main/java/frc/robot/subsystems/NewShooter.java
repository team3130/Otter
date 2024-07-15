// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewShooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX topShooterBar;
  private final WPI_TalonSRX bottomShooterBar;
  private double shooterVolts = 5;
  private double shooterToAmpVolts = 1;
  private final DigitalInput shooterBeam;
  private boolean flywheelsReachedSpeed = false;
  private final VoltageOut voltRequestTopBar = new VoltageOut(0);
  private final VoltageOut voltRequestBottomBar = new VoltageOut(0);
  private final VelocityVoltage topVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0).withSlot(1);
  public NewShooter() {
    topShooterBar = new WPI_TalonSRX(Constants.CAN.shooterTopFlywheel);
    bottomShooterBar = new WPI_TalonSRX(Constants.CAN.shooterBottomFlywheel);
    shooterBeam = new DigitalInput(Constants.IDs.shooterBeamDIO);
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
