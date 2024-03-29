// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SoftwareDeprecated.Camera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class JuhaeSnapToArcPID extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final JuhaeCameraSubsystem cameraSubsystem;
  private final Chassis chassis;

  /**
   * Creates a new ExampleCommand.
   *
   * @param cameraSubsystem The subsystem used by this command.
   */
  public JuhaeSnapToArcPID(JuhaeCameraSubsystem cameraSubsystem, Chassis chassis) {
    this.cameraSubsystem = cameraSubsystem;
    this.chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cameraSubsystem, chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cameraSubsystem.getLastEstimatedRobotToTarget();
    cameraSubsystem.getVectorSetpointSnapToArc();
    cameraSubsystem.getSnapToTargetSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    cameraSubsystem.resetSnapToDistanceController();
    Translation2d powerToSnap =
    chassis.driveTranslation2D();

    double x = chassis.goToDistancePower();
    theta = camera.goToFaceTargetPower();
    /*
    execute profiled PID
            drive (translation2D (pid outbut, getSnapToTargetSetpoint), thetaToFaceRobot);

     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
