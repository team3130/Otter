// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

/** A command to zero wheels of chassis */
public class ZeroWheels extends Command {
  private final Chassis chassis;

  public ZeroWheels(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  //Called when the scheduler starts the command
  @Override
  public void initialize() {
  }

  // Sets the angle PID controller to 0 degrees and calculates output for the motors
  @Override
  public void execute() {
    chassis.turnToAngle(90);
  }

  /**
   * Called when the scheduler ends the command.
   * Stops the chassis motors
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    chassis.stopModules();
  }

  /**
   * @return if the wheel PID controller is done. AKA are wheels zero-ed
   */
  @Override
  public boolean isFinished() {
    return chassis.turnToAnglePIDIsDone();
  }
}
