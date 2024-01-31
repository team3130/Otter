// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.subsystems.ClimberLeft;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberExtendLeft extends Command {
  private final ClimberLeft climber;
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberExtendLeft(ClimberLeft climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.brokeLeft() && speed < 0) {
        speed = 0;
    }

    climber.setSpeedLeft(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopLeft();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}