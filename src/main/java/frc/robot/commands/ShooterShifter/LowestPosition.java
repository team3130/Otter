// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterShifter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NewShooterShifter;

/** An example command that uses an example subsystem. */
public class LowestPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NewShooterShifter shooterShifter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterShifter The subsystem used by this command.
   */
  public LowestPosition(NewShooterShifter shooterShifter) {
    this.shooterShifter = shooterShifter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterShifter.goLowPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
