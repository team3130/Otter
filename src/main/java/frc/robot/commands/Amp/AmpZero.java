// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NewAmp;

/** An example command that uses an example subsystem. */
public class AmpZero extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NewAmp amp;
  private boolean raising = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param amp The subsystem used by this command.
   */
  public AmpZero(NewAmp amp) {
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.trackMotorDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(amp.getAmpLimit()){
      amp.trackMotorStop();
      amp.setAmpZeroed(true);
      amp.resetAmpEncoder();
    }
    if(amp.isAmpZeroed()){
      amp.trackMotorUp();
      raising = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.trackMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return amp.getAmpLocation() <= 500;
  }
}
