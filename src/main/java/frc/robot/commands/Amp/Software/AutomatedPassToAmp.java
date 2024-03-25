// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp.Software;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterShifter;

/** An example command that uses an example subsystem. */
public class AutomatedPassToAmp extends Command {
  private final Amp amp;
  private final Shooter shooter;
  private final Indexer indexer;
  private final ShooterShifter shifter;
  private boolean hasSeenNote = false;
  private Timer timer = new Timer();

  /**
   * @param amp The subsystem used by this command.
   */
  public AutomatedPassToAmp(Amp amp, Shooter shooter, Indexer indexer, ShooterShifter shifter) {
    this.amp = amp;
    this.shooter = shooter;
    this.indexer = indexer;
    this.shifter = shifter;
    addRequirements(shooter, indexer, shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeenNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shifter.getIsShortShifterExtended() && amp.getIsMid()) {
      shooter.runFlywheelsAmpSpeed();
      amp.outtakeAmp();
      indexer.toShooterSpindex();
    }
    if (shooter.getBeamHasNote() && !hasSeenNote){
      hasSeenNote = true; //note that we saw the note for the first time
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
    amp.ampSpinningMotorStop();
    indexer.stopIndexer();
    if (!shooter.getBeamHasNote() && hasSeenNote) {
      amp.setIsReadyToScore(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.getBeamHasNote() && hasSeenNote; //had a note and now we cant see it means it has passed
  }
}
