
package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
public class AmpIntake extends Command {
  private final Amp amp;
  private Timer timer = new Timer();

  /**
   * @param amp The subsystem used by this command.
   */
  public AmpIntake(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.intakeAmp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*timer.reset();
    timer.start();
    amp.rumble(timer);*/
    amp.ampSpinningMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false;
  }

}
