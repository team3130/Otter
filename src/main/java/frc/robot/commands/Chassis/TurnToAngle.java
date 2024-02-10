// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that uses chassis to spin the wheels to an angle.
 */
public class TurnToAngle extends Command {
  private final Chassis m_chassis;
  private final double point;

  public TurnToAngle(Chassis subsystem, double point) {
    m_chassis = subsystem;
    addRequirements(subsystem);
    this.point = point;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_chassis.turnToAngle(point); // turns the wheels to an angle
  }

  /**
   * Stops the chassis drive motors
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  /**
   * @return if chassis turn to angle PID is done
   */
  @Override
  public boolean isFinished() {
    return m_chassis.turnToAnglePIDIsDone();
  }
}
