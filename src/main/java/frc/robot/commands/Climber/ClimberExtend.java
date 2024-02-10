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
        double power = xboxController.getRawAxis(climber.getJoystick()); //TODO CHECK IF INVERTED
        power = power * Math.abs(power);


        if (power < 0){
            climber.setInvalidInput(true); // LED red
            power = 0;
        } else{
            climber.setInvalidInput(false);
        }

        // checks if limit switch has been broken or if theyre trying to move the climber down
        if (climber.brokeLimit()) {
            power = 0; // LED green
        }

        //sets the speed of the motor
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