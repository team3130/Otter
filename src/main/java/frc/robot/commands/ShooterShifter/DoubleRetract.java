// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterShifter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterShifter;

/** An example command that uses an example subsystem. */
public class DoubleRetract extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterShifter shooterShifter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shifter The subsystem used by this command.
   */
  public DoubleRetract(ShooterShifter shifter) {
    shooterShifter = shifter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterShifter.doubleRetract();
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