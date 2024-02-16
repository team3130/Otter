// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.XboxControllerVibration;
public class EndgameVibrateDriver extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final XboxControllerVibration vibrate;

  public EndgameVibrateDriver(XboxControllerVibration subsystem) {
    vibrate = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TESTING VIBRATION");
    //vibrate.TimedVibrateOperator();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vibrate.VibrateDriver();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vibrate.StopVibrateDriver();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
