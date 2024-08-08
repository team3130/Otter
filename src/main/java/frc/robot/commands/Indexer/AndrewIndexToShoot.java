// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NewIndexer;
import frc.robot.subsystems.NewShooterShifter;
import frc.robot.subsystems.NewShooter;

public class AndrewIndexToShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NewIndexer indexer;
  private final NewShooterShifter shooterShifter;
  private final NewShooter shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param indexer The subsystem used by this command.
   */

  public AndrewIndexToShoot(NewIndexer indexer, NewShooterShifter shooterShifter, NewShooter shooter) {
    this.indexer = indexer;
    this.shooterShifter = shooterShifter;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.hasShooterReachedSpeed() && (shooterShifter.getLowPosition() || shooterShifter.getHighPosition())){
      indexer.spinIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}