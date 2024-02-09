// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class SmartIndex extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;

  public SmartIndex(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  // intake slowly
  @Override
  public void execute() {
    intake.slowTake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stoptake();
    intake.setIntakeHasNote(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getShooterBreakBeam();
  }
}
