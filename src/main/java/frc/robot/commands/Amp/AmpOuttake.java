// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import java.util.Timer;

/** An example command that uses an example subsystem. */
public class AmpOuttake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Amp amp;
  private final Timer ampTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param amp
   */
  public AmpOuttake(Amp amp) {
    this.amp = amp;
    ampTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  /**
  *
   */
  @Override
  public void initialize() {
    amp.primeAmp();
    ampTimer.reset();
    ampTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * Change the hasElapsed second number to however long it takes
   * amp to prime again
   */
  @Override
  public void execute() {
    if (ampTimer.hasElapsed(1)) {
      amp.outtakeAmp();
      ampTimer.stop();
    }
    if (!amp.getLimitSwitch()) {
      amp.motorStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.unPrimeAmp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
