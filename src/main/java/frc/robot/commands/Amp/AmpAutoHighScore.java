// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
public class AmpAutoHighScore extends InstantCommand {
  private final Amp amp;

  /**
   * @param amp The subsystem used by this command.
   */
  public AmpAutoHighScore(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.resetController();
    amp.setIsMid(false);
    amp.setIsHigh(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.moveAmpAtSpeed(amp.runController(amp.getHighSetpoint()));
    if (amp.isAtSetpointWithDeadband()) {
      amp.setIsHigh(true);
      amp.outtakeAmp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.ampLiftingMotorStop();
    amp.ampSpinningMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (amp.getLiftingEncoderPosition() >= amp.getHighSetpoint()) || (!amp.getHasZeroed()) || !amp.getIsReadyToScore();
  }
}
