// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.XboxControllerVibration;

/** An example command that uses an example subsystem. */
public class Handoff extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer index;
  private final Intake intake;
  private final XboxControllerVibration vibration;

  /**
   * Creates a new ExampleCommand.
   *
   * @param //subsystem The subsystem used by this command.
   */
  public Handoff(Indexer indexer, Intake intaker, XboxControllerVibration vibrate) {
    index = indexer;
    intake = intaker;
    vibration = vibrate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!index.getIndexerLimitSwitch1()){
      index.runIndexers();
      intake.DumbIntake();
    }
    else {
      index.stopIndexers();
      intake.Stoptake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vibration.TimedVibrateOperator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
