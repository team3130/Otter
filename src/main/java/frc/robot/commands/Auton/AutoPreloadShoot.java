// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class AutoPreloadShoot extends Command {
    private final IntakeShooter shooter;
    private Timer timer = new Timer();
    private Timer timer2 = new Timer();
    public AutoPreloadShoot(IntakeShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer2.reset();
        timer2.start();
        timer.reset();
        timer.start();
        shooter.rawRunShooterFlywheels();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)){
            shooter.autoShooterSpindex();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopShooters();
        shooter.stopIndexer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer2.hasElapsed(1)) {
            return true;
        } else {
            return false;
        }
    }
}
