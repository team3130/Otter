// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class TargetingPressed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CameraSubsystem cam;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TargetingPressed(CameraSubsystem subsystem) {
    cam = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cam.setTryingToTargetTrue();
    cam.resetTargetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cam.setTryingToTargetFalse();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}