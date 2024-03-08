// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class AutoShoot extends Command {
    private final IntakeShooter shooter;
    private Timer spinUpTime = new Timer();
    private Timer timer2 = new Timer();
    public AutoShoot(IntakeShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer2.reset();
        timer2.start();
        spinUpTime.reset();
        spinUpTime.start();
        shooter.runShooterFlywheels();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (spinUpTime.hasElapsed(1)){
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
        if (timer2.hasElapsed(1.5)) {
            return true;
        } else {
            return false;
        }
    }
}
