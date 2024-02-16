// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.Handoff;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class IntakeThroughIndexer extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Indexer index;
  private final Intake intake;
  /**
   * Creates a new ExampleCommand.
   *
   * @param //subsystem The subsystem used by this command.
   */
  public IntakeThroughIndexer(Indexer indexer, Intake intaker) {
    index = indexer;
    intake = intaker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
    addRequirements(intake);

    addCommands(new SmartSpintake(intake), new Handoff(index, intake));
  }
}
