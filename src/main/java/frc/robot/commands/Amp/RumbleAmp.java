// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

/** An example command that uses an example subsystem. */
public class RumbleAmp extends Command {
  private final Amp amp;
  private final XboxController controller;
  private Timer timer = new Timer();

  public RumbleAmp(Amp amp, XboxController controller) {
    this.amp = amp;
    this.controller = controller;
    addRequirements(amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.8);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*
    if (amp.getLimitSwitch()) {
      timer.start();
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.8);
    }

     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
