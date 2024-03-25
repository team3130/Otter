
package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
public class AmpZero extends InstantCommand {
  private final Amp amp;

  /**
   * @param amp The subsystem used by this command.
   */
  public AmpZero(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.manualAmpLowerDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.ampLiftingMotorStop();
    if (amp.getLimitSwitch()) {
      amp.resetEncoder();
      amp.setHasZeroedTrue();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return amp.getLimitSwitch();
  }
}
