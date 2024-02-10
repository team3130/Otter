// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ClimberExtend extends Command {
    private final Climber climber;
    private XboxController xboxController;
    private int joystickButton;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
    */
    public ClimberExtend(Climber subsystem, XboxController xboxController, int joystickButton) {
        climber = subsystem;
        this.xboxController = xboxController;
        this.joystickButton = joystickButton;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // reads joystick input as y
        double y = xboxController.getRawAxis(joystickButton);
        //this ensures the output is always positive, so the driver can only spin
        // the motor one way, because the motor is on a ratchet
        y = y * y;

        if (!climber.brokeLimit()) {
            climber.setIsReset(false);
        }

        // checks if limit switch has been broken
        if (climber.brokeLimit() && !climber.getIsReset()) {
            y = 0;
        }

        //sets the speed of the motor
        climber.setSpeed(y);
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