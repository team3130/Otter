// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class SmartSpintake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private boolean hasPiece = false;
  private boolean intakeIsDown = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param //subsystem The subsystem used by this command.
   */
  public SmartSpintake(Intake Intake) {
    intake = Intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeDown();
    intakeIsDown = true;
    intake.GroundIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getLimitSwitch() && !hasPiece){
      hasPiece = true;
      intake.resetEncoders();
    }
    if (hasPiece){
      if (intakeIsDown || intake.getPosition() >= intake.getMaxIntakeTicks()){
        intake.Stoptake();
        intake.intakeUp();
        intakeIsDown = false;
      }
      else {
        intake.gentleIntake();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.Stoptake();
    hasPiece = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
