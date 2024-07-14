// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterShifter;

public class AutoShortExtend extends Command {
  private final ShooterShifter shooterShifter;
  private final Timer timer = new Timer();
  public AutoShortExtend(ShooterShifter shooterShifter) {
    this.shooterShifter = shooterShifter;
    addRequirements(shooterShifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterShifter.extendShortShifter();
    timer.reset();
    timer.start();

    System.out.println("Command initialized: Short extend ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Command interrupted: Short extend ");
    }
    System.out.println("Command finished: Short extend ");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(0.5)) {
      return true;
    } else {
      return false;
    }
  }
}

 */