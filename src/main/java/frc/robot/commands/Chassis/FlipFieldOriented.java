// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;

/**
 * A command that flips between robot and field oriented control
 * Requires chassis
 */
public class FlipFieldOriented extends InstantCommand {
  private final Chassis chassis;

  public FlipFieldOriented(Chassis chassis) {
    this.chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  /**
   * Called once when the scheduler runs for the first time
   */
  @Override
  public void initialize() {
    chassis.flipFieldRelative();
  }
}
