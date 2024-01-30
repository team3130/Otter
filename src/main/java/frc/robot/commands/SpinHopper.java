// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

/** An example command that uses an example subsystem. */
public class SpinHopper extends Command {
  private final Hopper hopper;
  private final Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param hopper The subsystem used by this command.
   */
  public SpinHopper(Hopper hopper) {
    timer = new Timer();
    this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.spinHopper();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(2.0)) {
      timer.stop();
      return true;
    } else {
      return false;
    }
  }
}
