// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;


public class IntakeRumble extends Command {
  private final Intake intake;
  private final Timer timer =new Timer();
private boolean nevermind = false;
  public IntakeRumble(Intake Intake) {
    intake = Intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.getIntakeLimitSwitch()) {
      timer.reset();
      timer.start();
    }
    else {nevermind = true;}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDriverToRumble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   intake.stopDriverRumble();
   timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (nevermind || timer.hasElapsed(1)) {
      return true;
    } else {
      return false;
    }
  }
}
