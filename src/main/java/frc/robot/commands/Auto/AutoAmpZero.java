
package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
/*public class AutoAmpZero extends InstantCommand {
  private final Amp amp;
  private Timer timer = new Timer();*/
  /**
   * @param amp The subsystem used by this command.
   */
  /*public AutoAmpZero(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.manualAmpLowerDown();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

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
    return amp.getLimitSwitch() || timer.hasElapsed(5);
  }
}
*/