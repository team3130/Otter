// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class PitClimberForward extends Command {
    private final Climber climber;

    public PitClimberForward(Climber climber) {
      this.climber = climber;
      addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // starts the motors and timer
        climber.setClimberSpeed(0.5);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
