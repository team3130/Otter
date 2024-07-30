// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NewAmp;

/** An example command that uses an example subsystem. */
public class AmpHighSetpoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NewAmp amp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param amp The subsystem used by this command.
   */
  public AmpHighSetpoint(NewAmp amp) {
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.resetAmpController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.moveAmpAtSpeed(amp.runAmpController(amp.getHighSetpoint()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
