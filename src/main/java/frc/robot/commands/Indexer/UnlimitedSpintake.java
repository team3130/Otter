// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewIndexer;
import frc.robot.subsystems.NewIntake;

/** An example command that uses an example subsystem. */
public class UnlimitedSpintake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NewIndexer indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param indexer The subsystem used by this command.
   */
  public UnlimitedSpintake(NewIndexer indexer) {
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.spinIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
