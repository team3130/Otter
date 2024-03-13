// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class VeloFlywheelsAndInitialIndex extends Command {
  private final Shooter shooter;
  private final Indexer indexer;

  private Timer spinUpTime = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter  The subsystem used by this command.
   */
  public VeloFlywheelsAndInitialIndex(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(shooter);
    addRequirements(indexer);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooter.getBeamHasNote()){
      indexer.spintake();
    }

    //spin up the flywheels
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
    indexer.stopIndexer();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}