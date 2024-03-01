// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class PitClimberReset extends Command {
    private final Climber climber;

    public PitClimberReset(Climber climber) {
      this.climber = climber;
      addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Constants.pitMode) {
            climber.setClimberSpeed(-0.7);
            climber.setIsClimberReset(false);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.stop();
        climber.setIsClimberReset(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (climber.brokeLimit()) {
            return true;
        } else {
            return false;
        }

    }
}
