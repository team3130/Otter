// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class DisableTargeting extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CameraSubsystem camera;
  private final Chassis chassis;
  private final CameraSubsystem cam;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DisableTargeting(Chassis subsystem, CameraSubsystem camSubsystem) {
    chassis = subsystem;
    cam = camSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cam.setTryingToTargetFalse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
