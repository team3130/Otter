// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterShifter;

public class AndrewIndex extends Command {
  private final Indexer indexer;
  private final ShooterShifter shooterShifter;
  private final Shooter shooter;

  public AndrewIndex(Indexer indexer, ShooterShifter shooterShifter, Shooter shooter) {
    this.indexer = indexer;
    this.shooterShifter = shooterShifter;
    this.shooter = shooter;
    addRequirements(indexer, shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getFlywheelsAtVelocitySetpoint() && (shooterShifter.getIsShortShifterExtended() || shooterShifter.getIsDoubleExtended())) {
    if ((shooterShifter.getIsShortShifterExtended() || shooterShifter.getIsDoubleExtended()) && indexer.flywheelVelocitiesReady()) {
      indexer.shooterSpindex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!indexer.flywheelVelocitiesReady()) {
      return true;
    } else {
      return false;
    }
  }
}
