// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SoftwareDeprecated.Testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** A default command to drive in teleop based off the joysticks
public class DriveVelocityTuning extends Command {
  private final VelocityChassis chassis;
  public DriveVelocityTuning(VelocityChassis chassis) {
    this.chassis = chassis;
    m_requirements.add(chassis);
  }

  /**
   * Called when the scheduler first schedules the command

  @Override
  public void initialize() {
  }

  /**
   * Called periodically while the default command is being ran and is not actively interrupted.
   * Takes the Joysticks inputs, applies a slew rate limit on it in meters per second which makes the input whooshier.
   * Inverts controls if we are on the red alliance because april tags give us an absolute position of the field

  @Override
  public void execute() {
    if (Constants.debugMode) {
      chassis.driveTuningVelocity();
    }
  }



  /**
   * Called when the command is over.
   * Stops the chassis modules
   * @param interrupted whether the command was interrupted/canceled

  @Override
  public void end(boolean interrupted) {
    chassis.stopModules();
  }

  /**
   * @return false. Never is finished.

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/