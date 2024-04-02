// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.ShooterShifter;

/** An example command that uses an example subsystem. */
public class AmpKirbyPrepMid extends Command {
  private final Amp amp;
  private final ShooterShifter shifter;

  /**
   * @param amp The subsystem used by this command.
   */
  public AmpKirbyPrepMid(Amp amp, ShooterShifter shifter) {
    this.amp = amp;
    this.shifter = shifter;
    addRequirements(amp, shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shifter.extendShortShifter();
    amp.resetController();
    amp.setIsMid(false);
    amp.setIsHigh(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.moveAmpAtSpeed(amp.runController(amp.getMidSetpoint()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.ampLiftingMotorStop();
    if (amp.isAtSetpointWithDeadband()) {
      amp.setIsMid(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!amp.getHasZeroed() || amp.isAtSetpoint());
  }
}
