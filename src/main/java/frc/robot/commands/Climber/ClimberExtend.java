// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ClimberExtend extends Command {
    private final Climber climber;
    private XboxController xboxController;
    private int joystickButton;

    public ClimberExtend(Climber side, XboxController xboxController) {
        climber = side;
        this.xboxController = xboxController;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // reads joystick input as y
        double power = -xboxController.getRawAxis(climber.getJoystick());
        power = power * Math.abs(power);


        if (!climber.brokeLimit()) {
            climber.setIsClimberReset(false);
        }

        // checks if limit switch has been broken
        if (climber.brokeLimit() && !climber.getIsClimberReset()) {
            power = 0;
            climber.setClimbDone(true);
        }

        if (power < 0.075) {
            power = 0;
        }
            climber.setClimberSpeed(power);
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