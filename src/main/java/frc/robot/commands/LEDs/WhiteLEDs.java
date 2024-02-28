// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDs.LEDs;

/** An example command that uses an example subsystem. */
public class WhiteLEDs extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDs leds;
  private double timeout;
  private Timer timer = new Timer();


  /**
   * Creates a new ExampleCommand.
   *
   * @param leds The subsystem used by this command.
   */
  public WhiteLEDs(LEDs leds, double timeout) {
    this.leds = leds;
    this.timeout = timeout;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leds.greenRobot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(timeout)) {
      return true;
    } else {
      return false;
    }
  }
}
