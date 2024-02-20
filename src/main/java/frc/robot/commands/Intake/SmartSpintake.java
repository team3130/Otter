// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class SmartSpintake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private Timer timer = new Timer();

  public SmartSpintake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // intake down, running at groundIntake speed
  @Override
  public void initialize() {
    timer.reset();
    intake.intakeDown();
    timer.start();
  }

  // once the limit switch is hit and we did not have a note, reset encoders and intake up
  @Override
  public void execute() {
    if (timer.hasElapsed(intake.getDropTime())){
      intake.groundIntake();
    }
    if (intake.getIntakeLimitSwitch() && !intake.getIntakeHasNote()){
      intake.setIntakeHasNote(true);
      intake.resetEncoders();
      intake.intakeUp();
      intakeIsDown = false;
    }

    // if we have a piece, slow intake
    if (intake.getIntakeHasNote()) {
      intake.slowTake();
    }
  }

  // stop the note
  @Override
  public void end(boolean interrupted) {
    intake.stoptake();
  }

  // end this command once the note is at its desired place to stop (via encoders)
  @Override
  public boolean isFinished() {
    if (intake.getEncoderPosition() >= intake.getIntakeNoteSetpoint()) {
      return true;
    }
    return false;
  }
}
