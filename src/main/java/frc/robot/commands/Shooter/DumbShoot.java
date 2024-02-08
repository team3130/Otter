// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DumbShoot extends Command {
    private final Shooter m_shooter;
    //private final Intake m_intake;
    private Timer spinUpTime = new Timer();
    public DumbShoot(Shooter shooter) {
        m_shooter = shooter;
        //m_intake = intake;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        spinUpTime.reset();
        spinUpTime.start();
        m_shooter.runShooters();
        //m_intake.runIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (spinUpTime.hasElapsed(1)){
            m_shooter.runIndexers();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooters();
        m_shooter.stopIndexers();
       // m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
