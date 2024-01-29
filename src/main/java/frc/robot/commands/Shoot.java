// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private Timer spinUpTime = new Timer();
    public Shoot(Shooter shooter, Indexer index, Intake intake) {
        m_shooter = shooter;
        m_indexer = index;
        m_intake = intake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
        addRequirements(index);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        spinUpTime.reset();
        spinUpTime.start();
        m_shooter.runShooters();
        m_indexer.runIndexers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (spinUpTime.hasElapsed(2.5)){
            m_intake.runIntake();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooters();
        m_indexer.stopIndexers();
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
