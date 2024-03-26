// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AutoFlywheel extends Command {
  private final Shooter shooter;
  private Timer timer = new Timer();
  double topStartingPoint = 0;
  double bottomStartingPoint = 0;

  double topRPSToIncrease = 0;
  double bottomRPSToIncrease = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter  The subsystem used by this command.
   */
  public AutoFlywheel(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runShooterFlywheels();
    System.out.println("Command initialized: Flywheel");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted) {
      System.out.println("Command interrupted: Flywheel");
    }
    System.out.println("Command finished: Flywheel");
    shooter.stopShooters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}