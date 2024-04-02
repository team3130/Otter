// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** the amp has nom-ed a note like kirby and is able to fly **/
public class AmpKirbyFlies extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Amp amp;
  private final Shooter shooter;
  private final ShooterShifter shooterShifter;
  private final Indexer indexer;
  private boolean beamHasSeenNote = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param amp The subsystem used by this command.
   */
  public AmpKirbyFlies(Amp amp, Shooter shooter, ShooterShifter shooterShifter, Indexer indexer) {
    this.amp = amp;
    this.shooter = shooter;
    this.shooterShifter = shooterShifter;
    this.indexer = indexer;
    addRequirements(amp, shooter, shooterShifter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beamHasSeenNote = false;
    amp.resetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterShifter.getIsShortShifterExtended() && amp.getIsMid()) {
      shooter.runFlywheelsAmpSpeed();
      amp.outtakeAmp();
      indexer.toShooterSpindex();
    }

    if (shooter.getBeamHasNote() && !beamHasSeenNote) {
      beamHasSeenNote = true; // break beam sees note for the first time
    }

    if (!shooter.getBeamHasNote() && beamHasSeenNote) { // when amp nom-ed note
      shooter.stopShooters();
      indexer.stopIndexer();
      amp.ampSpinningMotorStop();
      // fly up to high
      amp.moveAmpAtSpeed(amp.runController(amp.getHighSetpoint()));
      if (amp.isAtSetpointWithDeadband()) {
        amp.setIsHigh(true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.ampLiftingMotorStop();
    amp.ampSpinningMotorStop();
    shooter.stopShooters();
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (amp.getLiftingEncoderPosition() >= amp.getHighSetpoint()) || (!amp.getHasZeroed());
  }
}
