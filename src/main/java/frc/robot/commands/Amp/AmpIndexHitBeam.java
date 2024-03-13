// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class AmpIndexHitBeam extends Command {
  private final Amp amp;
  private final Shooter shooter;
  private final Indexer indexer;
  private Timer timer = new Timer();

  /**
   * @param amp The subsystem used by this command.
   */
  public AmpIndexHitBeam(Amp amp, Shooter shooter, Indexer indexer) {
    this.amp = amp;
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(amp);
    addRequirements(shooter);
    addRequirements(indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runIndexSpeed();
    amp.intakeAmp();
    indexer.spintake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
    amp.ampSpinningMotorStop();
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return amp.getAmpBeamHasNote(); //had a note
  }

}
