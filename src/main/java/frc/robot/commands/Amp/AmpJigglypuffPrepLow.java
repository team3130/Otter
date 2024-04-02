// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterShifter;

/** the amp has nom-ed a note like kirby and is able to fly **/
public class AmpJigglypuffPrepLow extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Amp amp;
  private final Shooter shooter;
  private final ShooterShifter shooterShifter;
  private final Indexer indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param amp The subsystem used by this command.
   */
  public AmpJigglypuffPrepLow(Amp amp, Shooter shooter, ShooterShifter shooterShifter, Indexer indexer) {
    this.amp = amp;
    this.shooter = shooter;
    this.shooterShifter = shooterShifter;
    this.indexer = indexer;
    addRequirements(amp, shooter, shooterShifter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterShifter.extendShortShifter();
    amp.setBeamHasSeenNote(false);
    amp.resetController();
    amp.setIsMid(false);
    amp.setIsHigh(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterShifter.getIsShortShifterExtended()) {
      indexer.indexToBeam();
    }
    if (shooter.getBeamHasNote() && !amp.beamHasSeenNote()) {
      amp.setBeamHasSeenNote(true); // break beam sees note for the first time
      shooterShifter.doubleRetract();
    }
    if (shooterShifter.getIsDoubleRetracted() && amp.beamHasSeenNote()) {
      amp.moveAmpAtSpeed(amp.runController(amp.getExperimentalLowSetpoint()));
    }

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
