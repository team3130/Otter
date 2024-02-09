// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterShifter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterShifter;

public class VisionShift extends Command {
  private final ShooterShifter shooterShifter;
  public VisionShift(ShooterShifter shifter) {
    shooterShifter = shifter;
    addRequirements(shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (CameraSubsystem.getDistance())  {
      if (distance == firstGoalDistance) {
        shooterShifter.extendShifterOne();
      } else if (distance == secondGoalDistance) {
        shooterShifter.doubleExtend();
      }
    }
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
