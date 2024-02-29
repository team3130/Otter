// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class VisionVelocityShoot extends Command {
  private final Shooter shooter;
  private Timer spinUpTime = new Timer();

  /**
   * @param shooter  The subsystem used by this command.
   */
  public VisionVelocityShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinUpTime.reset();
    spinUpTime.start();

    /*
    shooter.configureVelocitySlots();
    shooter.updateVelocityPID();

    if (CameraSubsystem.hasTarget()) {
      shooter.setFlywheelVelocity();
    }

     */
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}