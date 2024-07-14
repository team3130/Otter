
package frc.robot.commands.Amp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.NewAmp;

/** An example command that uses an example subsystem. */
public class AmpZero extends InstantCommand {
  private final NewAmp amp;
  private Timer timer = new Timer();
  private boolean raising = false;
  /**
   * @param amp The subsystem used by this command.
   */
  public AmpZero(NewAmp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.trackMotorDown();
    timer.reset();
    timer.start();
    raising = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (amp.getAmpLimit()) {
      amp.resetAmpEncoder();
      amp.setAmpZeroed(true);
    }
    if (amp.isAmpZeroed()) {
      amp.trackMotorUp();
      raising = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.trackMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (raising && amp.getAmpLocation() >= 500) || timer.hasElapsed(7);
  }
}
