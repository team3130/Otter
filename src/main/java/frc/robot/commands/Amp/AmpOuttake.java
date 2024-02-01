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
  private final Timer primeTimer;
  private final Timer outtakeTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param amp
   */
  public AmpOuttake(Amp amp) {
    this.amp = amp;
    primeTimer = new Timer();
    outtakeTimer = new Timer();
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
    primeTimer.reset();
    outtakeTimer.reset();
    primeTimer.start();
    outtakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * Change the hasElapsed second number to however long it takes
   * amp to prime again
   *
   * if primetime has elapsed, ampMotor is allowed to eject because
   * the amp arm has primed (lifted)
   *
   * if outtaketime has elapsed and the limit switch is released,
   * ampMotor stopped because the note has been fully ejected
   */
  @Override
  public void execute() {
    if (primeTimer.hasElapsed(amp.getPrimeTime())) {
      amp.outtakeAmp();
      primeTimer.stop();
    }
    if (!amp.getLimitSwitch() && outtakeTimer.hasElapsed(amp.getOuttakeTime())) {
      amp.motorStop();
      outtakeTimer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.motorStop();
    amp.unPrimeAmp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
