// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberRight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class ClimberExtendRight extends Command {
    
    private final ClimberRight climber;
    public XboxController xboxController;
    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClimberExtendRight(ClimberRight subsystem, XboxController xboxController) {
        this.climber = subsystem;
        this.xboxController = xboxController;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double y = xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKY); // TODO
        y = y * Math.abs(y);
        
        if (climber.brokeLimit() && y < 0) {
            y = 0;
        }

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