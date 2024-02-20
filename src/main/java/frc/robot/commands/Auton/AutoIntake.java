// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private Timer timer = new Timer();

  public AutoIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // intake down, running at groundIntake speed
  @Override
  public void initialize() {
    intake.intakeDown();
    intake.groundIntake();
    timer.reset();
    timer.start();
  }

  // once the limit switch is hit and we did not have a note, reset encoders and intake up
  @Override
  public void execute() {
    /*
    if (timer.hasElapsed(1)) {
      intake.groundIntake();
    }
     */
  }

  // stop the note
  @Override
  public void end(boolean interrupted) {
    intake.stoptake();
    intake.intakeUp();
  }

  // end this command once the note is at its desired place to stop (via encoders)
  @Override
  public boolean isFinished() {
    if (intake.getIntakeLimitSwitch()) {
      return true;
    }
    return false;
  }
}
