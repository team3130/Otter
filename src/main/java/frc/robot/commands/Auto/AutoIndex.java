// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AutoIndex extends Command {
  private final Indexer indexer;
  private Timer timer = new Timer();
  private Shooter shooter;
  private boolean beamHasSeenNote = false;
  public AutoIndex(Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    beamHasSeenNote = false;
    timer.reset();
    timer.start();
    indexer.autoShooterSpindex();
    System.out.println("Command initialized: Index");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getBeamHasNote() && !beamHasSeenNote) {
      beamHasSeenNote = true; // break beam sees note for the first time
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();

    if (interrupted) {
      System.out.println("Command interrupted: Index");
    }
    System.out.println("Command finished: Index");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(1) || (!shooter.getBeamHasNote() && beamHasSeenNote)) {
      return true;
    } else {
      return false;
    }
  }
}