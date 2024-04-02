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
public class AmpJigglypuffFlies extends Command {
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
  public AmpJigglypuffFlies(Amp amp, Shooter shooter, ShooterShifter shooterShifter, Indexer indexer) {
    this.amp = amp;
    this.shooter = shooter;
    this.shooterShifter = shooterShifter;
    this.indexer = indexer;
    addRequirements(amp, shooter, shooterShifter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.resetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooter.getBeamHasNote() && amp.beamHasSeenNote()) { // when amp nom-ed note
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (amp.getLiftingEncoderPosition() >= amp.getHighSetpoint()) || (!amp.getHasZeroed());
  }
}
