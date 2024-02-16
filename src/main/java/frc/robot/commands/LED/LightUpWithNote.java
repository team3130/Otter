// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;

/** An example command that uses an example subsystem. */
public class LightUpWithNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem ledSubsystem;
  private final Amp amp;
  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param //subsystem The subsystem used by this command.
   */
  public LightUpWithNote(LEDSubsystem ledSubsystem, Amp amp, Intake intake) {
    this.ledSubsystem = ledSubsystem;
    this.amp = amp;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (amp.getLimitSwitch()) { //|| //intake.intakeLimitSwitch()) {
      ledSubsystem.green();
    }
    else
    {
      ledSubsystem.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
