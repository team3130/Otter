// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private Timer timer = new Timer();
  public AutoIntake(Intake intake, Indexer indexer) {
    this.intake = intake;
    this.indexer = indexer;
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeDown();
    timer.reset();
    timer.start();
    System.out.println("Command initialized: Intake " );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(1)) {
      indexer.autoSpintake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Command interrupted: Intake");
    }
    System.out.println("Command finished: Intake");
    indexer.stopIndexer();
    intake.intakeUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((intake.getIntakeLimitSwitch() && timer.hasElapsed(0.75)) || timer.hasElapsed(3)) {
      return true;
    } else {
      return false;
    }
  }
}