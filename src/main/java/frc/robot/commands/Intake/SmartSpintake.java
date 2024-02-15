// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.XboxControllerVibration;

/** An example command that uses an example subsystem. */
public class SmartSpintake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private final XboxControllerVibration vibrate;

  /**
   * Creates a new ExampleCommand.
   *
   * @param //subsystem The subsystem used by this command.
   */
  public SmartSpintake(Intake Intake, XboxControllerVibration vibration) {
    intake = Intake;
    vibrate = vibration;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SolenoidToggle();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.limitSwitchCheck();
    intake.SmartIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.Stoptake();
    intake.SolenoidToggle();
    vibrate.SmallTimedVibrateDriver();
    vibrate.SmallTimedVibrateOperator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
