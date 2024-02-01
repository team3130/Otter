// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberLeft;
import frc.robot.subsystems.Climber.ClimberRight;

/** An example command that uses an example subsystem. */
public class ResetClimberLeft extends Command {

    private final ClimberLeft climberLeft;

    private boolean isDone = false;

    private final Timer timer;

    public ResetClimberLeft(ClimberRight climberRight, ClimberLeft climberLeft) {
      this.climberLeft = climberLeft;
      timer = new Timer();
      addRequirements(climberRight);
      addRequirements(climberLeft);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // starts the motors and timer
        climberLeft.setSpeed(0); // TODO

        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(0.1)) {
            // checks if the voltage spiked
            if (climberLeft.getMotorCurrent() >= Constants.Climber.currentMax) {
                // inverts the motors
                climberLeft.invert();
            }
            isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climberLeft.stop();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isDone;
    }
}
