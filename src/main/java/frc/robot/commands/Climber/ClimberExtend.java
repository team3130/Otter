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
    private int limitsEncountered = 0;
    private boolean hasLeftLimit = false;

    public ClimberExtend(Climber side, XboxController xboxController) {
        climber = side;
        this.xboxController = xboxController;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {if (climber.brokeLimit()){
        climber.setIsClimberOnLimit(true);}
    else{climber.setIsClimberOnLimit(false);
    }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // reads joystick input as y
        double power = -xboxController.getRawAxis(climber.getJoystick());
        power = power * Math.abs(power);


        if (power < 0.075){
            climber.setInvalidInput(true);
            //leds go red
            power = 0;
        } else{
            climber.setInvalidInput(false);
        }

       if (climber.getIsClimberOnLimit()) {
           if (!climber.brokeLimit()){
               hasLeftLimit = true;
           }
           if (climber.brokeLimit() && hasLeftLimit){
               limitsEncountered++;
           }
            if (limitsEncountered >= 2){
                power = 0;
            }
        }
       if (!climber.getIsClimberOnLimit() && climber.brokeLimit()){
               power = 0;
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