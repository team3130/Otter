// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class ClimberExtendLeft extends Command {
  
    private final Climber climber;
    public XboxController xboxController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
    */
    public ClimberExtendLeft(Climber subsystem, XboxController xboxController) {
        climber = subsystem;
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
        double y = xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKY); // TODO
        y = y * Math.abs(y);

        // checks if limit switch has been broken
        if (climber.brokeLimit() && y < 0) {
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