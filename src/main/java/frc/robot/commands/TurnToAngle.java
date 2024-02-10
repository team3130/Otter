// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class TurnToAngle extends Command {
  private final Chassis chassis;
  private double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public TurnToAngle(Chassis chassis, double set) {
    this.chassis = chassis;
    this.setpoint = set;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.shuffleboardUpdatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.goToAnglePower(Math.toRadians(setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.goToAnglePIDIsFinished();
  }
}
