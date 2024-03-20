// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Indexer;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class IndexToBeam extends Command {
  private final Indexer indexer;
  private final Shooter shooter;

  private Timer spinUpTime = new Timer();

  public IndexToBeam(Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
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
    indexer.stopIndexer();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}