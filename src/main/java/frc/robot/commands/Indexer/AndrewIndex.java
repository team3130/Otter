// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.ShooterShifter;

public class AndrewIndex extends Command {
  private final IntakeShooter indexer;
  private final ShooterShifter shooterShifter;

  public AndrewIndex(IntakeShooter indexer, ShooterShifter shooterShifter) {
    this.indexer = indexer;
    this.shooterShifter = shooterShifter;
    addRequirements(indexer, shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterShifter.getIsShortShifterExtended() || shooterShifter.getIsDoubleExtended()) {
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
    return false;
  }
}
