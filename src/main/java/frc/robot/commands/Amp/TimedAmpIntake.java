// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
public class TimedAmpIntake extends Command {
  private final Amp amp;
  private final Timer timer = new Timer();

  /**
   * @param amp The subsystem used by this command.
   */
  public TimedAmpIntake(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.intakeAmp();
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
    amp.ampMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(1)) {
      return true;
    } else {
      return false;
    }
  }
}