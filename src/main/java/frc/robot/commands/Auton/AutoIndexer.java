// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class AutoIndexer extends Command {
    private final Indexer indexer;
    private Timer spinUpTime = new Timer();
    public AutoIndexer(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        spinUpTime.reset();
        spinUpTime.start();
        indexer.spintake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stoptake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (spinUpTime.hasElapsed(.75)) {
            return true;
        } else {
            return false;
        }
    }
}
