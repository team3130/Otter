// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class OnlyIndex extends Command {
  private final Shooter shooter;

  public OnlyIndex(Shooter index) {
    shooter = index;
    addRequirements( index);
  }

  @Override
  public void initialize() {
    shooter.runIndexers();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIndexers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
