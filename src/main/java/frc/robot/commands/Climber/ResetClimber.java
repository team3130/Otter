// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.*;

/** An example command that uses an example subsystem. */
public class ResetClimber extends Command {

    private final ClimberLeft climberLeft;

    private final ClimberRight climberRight;

    public ResetClimber(ClimberRight climberRight, ClimberLeft climberLeft) {
      this.climberRight = climberRight;
      this.climberLeft = climberLeft;
      addRequirements(climberRight);
      addRequirements(climberLeft);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        

        if ((climberRight.getMotorCurrent() >= Constants.Climber.currentMax) || (climberLeft.getMotorCurrent() >= Constants.Climber.currentMax)) {
            climberRight.invert();
            climberLeft.invert();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climberLeft.stop();
      climberRight.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
