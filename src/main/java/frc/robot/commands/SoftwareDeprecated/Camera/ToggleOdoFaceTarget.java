// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SoftwareDeprecated.Camera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class ToggleOdoFaceTarget extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis chassis;
  private final CameraSubsystem camera;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ToggleOdoFaceTarget(Chassis subsystem, CameraSubsystem cameraSubsystem) {
    chassis = subsystem;
    camera = cameraSubsystem;
    m_requirements.add(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
    chassis.setFaceTargetting(!chassis.isVectorFaceTargetting());
    chassis.vectorFaceAprilTag();
    camera.resetTargetController();

     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
    System.out.println(chassis.isVectorFaceTargetting());

     */

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
