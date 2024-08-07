// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.NewShooter;
import org.w3c.dom.ls.LSOutput;

public class VelocityShoot extends Command {
  private final NewShooter shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter  The subsystem used by this command.
   */
  public VelocityShoot(NewShooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.debugMode) {
      shooter.updateShooterPID();
      shooter.configureShooterConfigs();
    }
    shooter.setTryingToShoot(true);
    System.out.println("velocity shoot");
    shooter.setShooterVelocity();
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.checkShooterAtSetpoint();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.setTryingToShoot(false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}