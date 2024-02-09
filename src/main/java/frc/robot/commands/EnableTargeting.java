// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
<<<<<<<<< Temporary merge branch 1:src/main/java/frc/robot/commands/Chassis/FlipDriveOrientation.java
public class FlipDriveOrientation extends Command {
  private final Chassis chassis;
=========
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
>>>>>>>>> Temporary merge branch 2:src/main/java/frc/robot/commands/Shooter/IntakeCommand.java

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
<<<<<<<<< Temporary merge branch 1:src/main/java/frc/robot/commands/Chassis/FlipDriveOrientation.java
  public FlipDriveOrientation(Chassis subsystem) {
    this.chassis = subsystem;
=========
  public IntakeCommand(Intake subsystem) {
    intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
>>>>>>>>> Temporary merge branch 2:src/main/java/frc/robot/commands/Shooter/IntakeCommand.java
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
