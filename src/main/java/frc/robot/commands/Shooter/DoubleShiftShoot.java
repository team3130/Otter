// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterShifter;

public class DoubleShiftShoot extends Command {
  private final Shooter shooter;
  private final ShooterShifter shooterShifter;

  public DoubleShiftShoot(Shooter shooter, ShooterShifter shooterShifter) {
    this.shooter = shooter;
    this.shooterShifter = shooterShifter;
    addRequirements(shooter, shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runShooters();
    shooterShifter.doubleExtend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
