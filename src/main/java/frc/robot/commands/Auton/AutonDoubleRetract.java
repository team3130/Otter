// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterShifter;

public class AutonDoubleRetract extends InstantCommand {
  private final ShooterShifter shooterShifter;
  private final Timer timer = new Timer();

  public AutonDoubleRetract(ShooterShifter shifter) {
    shooterShifter = shifter;
    addRequirements(shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterShifter.doubleRetract();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(0.5)) {
      return true;
    } else {
      return false;
    }
  }
}
